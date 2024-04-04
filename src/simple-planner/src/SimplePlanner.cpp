/**
 * @file SimplePlanner.cpp
 * @author Mohamed Elobaid
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#include <fstream>
#include <optional>

#include <yarp/sig/Vector.h>

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <SimplePlanner/SimplePlanner.h>

using namespace StableCentroidalMPCWalking;

struct SimplePlanner::Impl
{

    bool saveData{true};

    std::ofstream file;

    BipedalLocomotion::Contacts::ContactListMap m_contactListMap;

    UnicycleGenerator m_trajectoryGenerator;
    UnicycleController m_unicycleController;
    std::shared_ptr<FeetGenerator> m_feetGenerator;
    std::shared_ptr<DCMTrajectoryGenerator> m_dcmGenerator;

    std::vector<bool> leftInContactPeriod;
    std::vector<bool> rightInContactPeriod;

    enum class FSM
    {
        Idle,
        Initialized,
        FirstStep,
        OutputValid,
        OutputInvalid
    };

    FSM m_generatorState{FSM::Idle};

    std::shared_ptr<UnicyclePlanner> m_unicyclePlanner = m_trajectoryGenerator.unicyclePlanner();
    std::shared_ptr<FootPrint> m_leftFootPrint, m_rightFootPrint;


    std::optional<BipedalLocomotion::Contacts::PlannedContact> m_InitleftStep, m_InitrightStep;

    StepList m_leftSteps, m_rightSteps;


    double m_time;
    double m_plannerPeriod, m_plannerHorizon;

    iDynTree::Vector2 m_currentUnicyclePosition;
    iDynTree::Vector2 m_unicycleControlInput;

    yarp::os::BufferedPort<yarp::sig::Vector> m_inputPort;
    std::string m_joypadCommandPortName;
    iDynTree::Vector2 m_desiredTrajectoryPoint;

    std::vector<iDynTree::Vector2> m_dcmPositions;
    std::vector<iDynTree::Vector2> m_dcmVelocities;

    bool configurePlanner(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
    {

        m_unicyclePlanner = std::make_shared<UnicyclePlanner>();

        auto handlerPtr = handler.lock();
        if (handlerPtr == nullptr)
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] The "
                                            "handler is not valid.");
            return false;
        }

        auto ptr = handlerPtr->getGroup("PLANNER_PARAMETERS").lock();

        if (ptr == nullptr)
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] Unable "
                                            "to get the planner parameters.");
            return false;
        }

        iDynTree::Vector2 refOffset, saturation;
        if (!ptr->getParameter("referencePosition", refOffset)
            || !ptr->getParameter("saturationFactors", saturation))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] Unable "
                                            "to get the reference position or the saturation "
                                            "factors.");
            return false;
        }

        if (!ptr->getParameter("joypadCommandPortName", m_joypadCommandPortName))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] Unable "
                                            "to get the joypad command port name.");
            return false;
        }

        double controllerGain, dt, maxStepLength, minStepLength, minStepWidth, maxAngleVariation,
            minAngleVariation, nominalWidth, positionWeight, timeWeight, minStepDuration,
            maxStepDuration, stepHeight, nominalDuration, slowWhenTurnGain;

        // Retrieve parameters from the parameter handler
        if (!ptr->getParameter("unicycleGain", controllerGain)
            || !ptr->getParameter("plannerIntgrationStep", dt)
            || !ptr->getParameter("maxStepLength", maxStepLength)
            || !ptr->getParameter("minStepLength", minStepLength)
            || !ptr->getParameter("minWidth", minStepWidth)
            || !ptr->getParameter("maxAngleVariation", maxAngleVariation)
            || !ptr->getParameter("minAngleVariation", minAngleVariation)
            || !ptr->getParameter("nominalWidth", nominalWidth)
            || !ptr->getParameter("positionWeight", positionWeight)
            || !ptr->getParameter("timeWeight", timeWeight)
            || !ptr->getParameter("minStepDuration", minStepDuration)
            || !ptr->getParameter("maxStepDuration", maxStepDuration)
            || !ptr->getParameter("stepHeight", stepHeight)
            || !ptr->getParameter("nominalDuration", nominalDuration)
            || !ptr->getParameter("plannerIntgrationStep", m_plannerPeriod)
            || !ptr->getParameter("plannerHorizon", m_plannerHorizon)
            || !ptr->getParameter("slowWhenTurningGain", slowWhenTurnGain))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] Unable "
                                            "to get one or more parameters for the planner.");
            return false;
        }

        // Configure the planner
        if (!m_unicyclePlanner->setDesiredPersonDistance(refOffset(0), refOffset(1))
            || !m_unicyclePlanner->setPersonFollowingControllerGain(controllerGain)
            || !m_unicyclePlanner->setMaximumIntegratorStepSize(dt)
            || !m_unicyclePlanner->setMaxStepLength(maxStepLength, 0.8)
            || !m_unicyclePlanner->setWidthSetting(minStepWidth, nominalWidth)
            || !m_unicyclePlanner->setMaxAngleVariation(maxAngleVariation)
            || !m_unicyclePlanner->setCostWeights(positionWeight, timeWeight)
            || !m_unicyclePlanner->setStepTimings(minStepDuration, maxStepDuration, nominalDuration)
            || !m_unicyclePlanner->setPlannerPeriod(dt)
            || !m_unicyclePlanner->setMinimumAngleForNewSteps(minAngleVariation)
            || !m_unicyclePlanner->setMinimumStepLength(minStepLength)
            || !m_unicyclePlanner->setSlowWhenTurnGain(slowWhenTurnGain)
            || !m_unicyclePlanner->setSaturationsConservativeFactors(saturation(0), saturation(1)))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] Unable "
                                            "to configure the planner.");
            return false;
        }

        std::string controllerType;
        if (!ptr->getParameter("controllerType", controllerType))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] Unable "
                                            "to get the controller type.");
            return false;
        }

        if (controllerType == "personFollowing")
        {
            m_unicycleController = UnicycleController::PERSON_FOLLOWING;
        } else if (controllerType == "direct")
        {
            m_unicycleController = UnicycleController::DIRECT;
        } else
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] "
                                            "Invalid "
                                            "controller type.");
            return false;
        }

        if (!m_unicyclePlanner->setUnicycleController(m_unicycleController))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] Unable "
                                            "to set the controller type.");
            return false;
        }
        m_feetGenerator = m_trajectoryGenerator.addFeetMinimumJerkGenerator();
        m_feetGenerator->setStepHeight(stepHeight);


        // configure the dcm generator
        iDynTree::Vector2 leftZMPDelta, rightZMPDelta;
        double lastStepDCMOffset;
        double comHeight;

        if (!ptr->getParameter("leftZMPDelta", leftZMPDelta)
            || !ptr->getParameter("rightZMPDelta", rightZMPDelta)
            || !ptr->getParameter("lastStepDCMOffset", lastStepDCMOffset)
            || !ptr->getParameter("comHeight", comHeight))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] Unable "
                                            "to get the DCM parameters.");
            return false;
        }

        m_dcmGenerator = m_trajectoryGenerator.addDCMTrajectoryGenerator();

        m_dcmGenerator->setFootOriginOffset(leftZMPDelta, rightZMPDelta);
        m_dcmGenerator->setOmega(sqrt(9.81/comHeight));
        m_dcmGenerator->setFirstDCMTrajectoryMode(FirstDCMTrajectoryMode::FifthOrderPoly);

        if (!m_dcmGenerator->setLastStepDCMOffsetPercentage(lastStepDCMOffset))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] Unable "
                                            "to set the last step DCM offset.");
            return false;
        }

        DCMInitialState dcmInitialState;
        dcmInitialState.initialPosition.zero();
        dcmInitialState.initialVelocity.zero();

        if (!m_dcmGenerator->setDCMInitialState(dcmInitialState))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configurePlanner] Unable "
                                            "to set the initial DCM state.");
            return false;
        }

        // TODO: remove me to config file
        m_unicyclePlanner->addTerminalStep(true);
        m_unicyclePlanner->startWithLeft(true);

        m_currentUnicyclePosition.zero();
        m_unicycleControlInput.zero();




        m_trajectoryGenerator.setSwitchOverSwingRatio(0.3); // TODO, remove me to config file
        m_trajectoryGenerator.setPauseConditions(maxStepDuration, nominalDuration);
        m_trajectoryGenerator.setTerminalHalfSwitchTime(1.0);

        m_time = 0.0;

        return true;
    }

    bool configure()
    {
        if (!m_inputPort.open("/contacts-list-provider/joypad-input"))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configure] Unable to "
                                            "open the input port.");
            return false;
        }

        if (!yarp::os::Network::connect(m_joypadCommandPortName, m_inputPort.getName()))
        {
            BipedalLocomotion::log()->warn("[SimplePlanner::Impl::configure] Unable to "
                                           "connect to the joypad command port.");
        }

        file.open("contact-list-provider-output.txt");
        if (!file.is_open())
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::configure] Unable to "
                                            "open the file.");
            return false;
        }

        return true;
    }

    bool updatePlannerDesiredTrajectory()
    {
        yarp::sig::Vector* command = m_inputPort.read(false);
        if (command == nullptr)
        {
            BipedalLocomotion::log()->warn("[SimplePlanner::Impl::"
                                           "updatePlannerDesiredTrajectory] "
                                           "Unable to read the joypad command.");
            return false;
        }

        m_desiredTrajectoryPoint[0] = (*command)[0];
        m_desiredTrajectoryPoint[1] = (*command)[1];

        if (m_unicycleController == UnicycleController::PERSON_FOLLOWING)
        {
            auto time = 0.0;
            m_currentUnicyclePosition.zero();
            m_unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(time,
                                                                        m_currentUnicyclePosition);

            m_unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(time,
                                                                        m_desiredTrajectoryPoint);
        } else if (m_unicycleController == UnicycleController::DIRECT)
        {
            m_unicyclePlanner->setDesiredDirectControl(m_desiredTrajectoryPoint[0],
                                                       m_desiredTrajectoryPoint[1],
                                                       0.0);
        }

        return true;
    }

    bool computeSteps()
    {

        m_leftFootPrint = std::make_shared<FootPrint>();
        m_rightFootPrint = std::make_shared<FootPrint>();
        m_leftFootPrint->setFootName("left");
        m_rightFootPrint->setFootName("right");

        double initialTime = m_time;
        if (m_InitleftStep.has_value())
        {
            const auto& contact = m_InitleftStep;

            const auto& euler
                = contact->pose.quat().normalized().toRotationMatrix().eulerAngles(1, 0, 2);

            iDynTree::Vector2 translation;
            translation[0] = contact->pose.translation()[0];
            translation[1] =  contact->pose.translation()[1];

            m_leftFootPrint->addStep(translation,
                                euler[2],
                                std::chrono::duration<double>(contact->activationTime).count());

            const double impactTime = std::chrono::duration<double>(contact->activationTime).count();
            initialTime = impactTime > initialTime ? impactTime : initialTime;
        }

        if (m_InitrightStep.has_value())
        {
            const auto& contact = m_InitrightStep;

            const auto& euler
                = contact->pose.quat().normalized().toRotationMatrix().eulerAngles(1, 0, 2);

            iDynTree::Vector2 translation;
            translation[0] = contact->pose.translation()[0];
            translation[1] =  contact->pose.translation()[1];
            m_rightFootPrint->addStep(translation,
                                euler[2],
                                std::chrono::duration<double>(contact->activationTime).count());

            const double impactTime = std::chrono::duration<double>(contact->activationTime).count();
            initialTime = impactTime > initialTime ? impactTime : initialTime;
        }

        auto finalTime = initialTime + m_plannerHorizon;
        if (!m_unicyclePlanner->computeNewSteps(m_leftFootPrint,
                                                m_rightFootPrint,
                                                initialTime,
                                                finalTime))
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::computeNewSteps] Unable "
                                            "to compute new steps.");
            return false;
        }

        m_leftSteps = m_leftFootPrint->getSteps();
        m_rightSteps = m_rightFootPrint->getSteps();


        return true;
    }

    bool generateTrajectories()
    {

        m_feetGenerator = m_trajectoryGenerator.addFeetMinimumJerkGenerator();

        m_dcmGenerator = m_trajectoryGenerator.addDCMTrajectoryGenerator();

        const double startLeft = m_leftSteps.front().impactTime;
        const double startRight = m_rightSteps.front().impactTime;
        const double start = std::max(startLeft, startRight);

        if (!m_trajectoryGenerator.generateFromFootPrints(m_leftFootPrint,
                                                          m_rightFootPrint,
                                                          start,
                                                          0.1)) // horizon = 20
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::generateTrajectories] "
                                            "Unable to generate the feet trajectory.");
            return false;
        }

        m_trajectoryGenerator.getFeetStandingPeriods(leftInContactPeriod, rightInContactPeriod);

        m_dcmPositions =  m_dcmGenerator->getDCMPosition();
        if (m_dcmPositions.empty())
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::generateTrajectories] "
                                            "Unable to get the DCM positions.");
        }

        m_dcmVelocities = m_dcmGenerator->getDCMVelocity();
        if (m_dcmVelocities.empty())
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::generateTrajectories] "
                                            "Unable to get the DCM velocities.");
        }


        return true;
    }


    std::vector<BipedalLocomotion::Contacts::PlannedContact>
    getPlannedContactFromStep(const std::vector<bool>& isFootInContactVector, const StepList& steps)
    {

        constexpr double secondsToNanoseconds = 1e9;
        std::vector<BipedalLocomotion::Contacts::PlannedContact> contacts;

        for (const auto& step : steps)
        {
            BipedalLocomotion::Contacts::PlannedContact contact;
            contact.name = step.footName;
            contact.activationTime = std::chrono::nanoseconds(
                static_cast<long long>(step.impactTime * secondsToNanoseconds));
            contact.deactivationTime = contact.activationTime;
            contacts.push_back(contact);
        }

        double ContactTime = 0.0;
        size_t contactIt = 0;

        for (size_t it = 1; it < isFootInContactVector.size(); ++it)
        {

            auto& contact = contacts[contactIt];

            const bool thisState = isFootInContactVector[it];
            const bool lastState = isFootInContactVector[it - 1];

            ContactTime += m_plannerPeriod;

            if (lastState == 0 && thisState == 1)
                ContactTime = 0.0;

            if (lastState == 1 && thisState == 0)
            {
                using namespace std::chrono_literals;
                contact.deactivationTime
                    = contact.activationTime
                      + std::chrono::duration_cast<std::chrono::nanoseconds>(ContactTime * 1s);
                contactIt++;
            }
        }

        contacts.back().deactivationTime = std::chrono::nanoseconds::max();

        return contacts;
    }


    bool computeContactList()
    {

        std::vector<BipedalLocomotion::Contacts::PlannedContact> leftContacts
            = getPlannedContactFromStep(leftInContactPeriod, m_leftSteps);
        std::vector<BipedalLocomotion::Contacts::PlannedContact> rightContacts
            = getPlannedContactFromStep(rightInContactPeriod, m_rightSteps);

        if (m_leftSteps.size() != leftContacts.size()
            || m_rightSteps.size() != rightContacts.size())
        {
            BipedalLocomotion::log()->error("[SimplePlanner::Impl::computeContactList] "
                                            "Mismatch between the number of steps and the number "
                                            "of "
                                            "contacts.");
            return false;
        }

        assert(leftContacts.size() == m_leftSteps.size());
        for (size_t i = 0; i < leftContacts.size(); ++i)
        {
            leftContacts[i].pose.quat(
                Eigen::AngleAxisd(m_leftSteps[i].angle, Eigen::Vector3d::UnitZ()));
            leftContacts[i].pose.translation(
                {m_leftSteps[i].position[0], m_leftSteps[i].position[1], 0.0});
        }

        assert(rightContacts.size() == m_rightSteps.size());
        for (size_t i = 0; i < rightContacts.size(); ++i)
        {
            rightContacts[i].pose.quat(
                Eigen::AngleAxisd(m_rightSteps[i].angle, Eigen::Vector3d::UnitZ()));
            rightContacts[i].pose.translation(
                {m_rightSteps[i].position[0], m_rightSteps[i].position[1], 0.0});
        }

        BipedalLocomotion::Contacts::ContactList leftContactList;
        BipedalLocomotion::Contacts::ContactList rightContactList;
        for (const auto& contact : leftContacts)
        {
            leftContactList.addContact(contact);
        }

        for (const auto& contact : rightContacts)
        {
            rightContactList.addContact(contact);
        }

        m_contactListMap["left_foot"] = leftContactList;
        m_contactListMap["right_foot"] = rightContactList;

        // update inital contacts with last planned contact
        m_InitleftStep = leftContacts.back();
        m_InitrightStep = rightContacts.back();

        return true;
    }

    void resetPlanner()
    {
        m_leftFootPrint = nullptr;
        m_rightFootPrint = nullptr;
        m_contactListMap.clear();
        m_dcmPositions.clear();
        m_dcmVelocities.clear();
    }

    void saveDataToFile(const BipedalLocomotion::Contacts::ContactListMap& contactListMap)
    {

        for (const auto& contactList : contactListMap)
        {
            file << "Foot: " << contactList.first << std::endl;
            for (const auto& contact : contactList.second)
            {
                file << "Contact: " << contact.name << std::endl;
                file << "Activation time: " << contact.activationTime.count() << std::endl;
                file << "Deactivation time: " << contact.deactivationTime.count() << std::endl;
                file << "Pose: " << contact.pose.translation().transpose() << std::endl;
                file << "Orientation: " << contact.pose.quat().coeffs().transpose() << std::endl;
            }
        }

        // save dcm positions
        for (const auto& dcm : m_dcmPositions)
        {
            file << "DCM: " << dcm(0) << " " << dcm(1) << std::endl;
        }
    }

    ~Impl()
    {
        m_inputPort.close();
        if (file.is_open())
        {
            file.close();
        }
    }
};

SimplePlanner::SimplePlanner()
{
    m_pimpl = std::make_unique<Impl>();
}

SimplePlanner::~SimplePlanner() = default;

bool SimplePlanner::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    if (!m_pimpl->configurePlanner(handler))
    {
        BipedalLocomotion::log()->error("[SimplePlanner::initialize] Unable to configure "
                                        "the planner.");
        return false;
    }

    if (!m_pimpl->configure())
    {
        BipedalLocomotion::log()->error("[SimplePlanner::initialize] Unable to configure "
                                        "the contacts list provider.");
        return false;
    }

    m_pimpl->m_generatorState = Impl::FSM::Initialized;

    return true;
}

bool SimplePlanner::isOutputValid() const
{
    return m_pimpl->m_generatorState == Impl::FSM::OutputValid;
}

bool SimplePlanner::advance()
{
    constexpr auto logPrefix = "[SimplePlanner::advance]";

    if (!m_pimpl->updatePlannerDesiredTrajectory())
    {
        BipedalLocomotion::log()->warn("{} Unable to update the planner desired trajectory.",
                                       logPrefix);
    }

    if (!m_pimpl->computeSteps())
    {
        BipedalLocomotion::log()->warn("{} Unable to compute the steps.", logPrefix);
    }

    if (!m_pimpl->generateTrajectories())
    {
        BipedalLocomotion::log()->warn("{} Unable to generate the trajectories.", logPrefix);
    }

    if (!m_pimpl->computeContactList())
    {
        BipedalLocomotion::log()->warn("{} Unable to compute the contact list.", logPrefix);
    }

    // reset planner and update time
    m_pimpl->m_time += m_pimpl->m_plannerPeriod;

    if (m_pimpl->saveData)
    {
        m_pimpl->saveDataToFile(m_pimpl->m_contactListMap);
    }

    // set the output
    m_output.contactList = m_pimpl->m_contactListMap;
    m_output.dcmPositions = m_pimpl->m_dcmPositions;
    m_output.dcmVelocities = m_pimpl->m_dcmVelocities;

    m_pimpl->resetPlanner();

    return true;
}

const SimplePlanner::Output& SimplePlanner::getOutput() const
{
    return m_output;
}