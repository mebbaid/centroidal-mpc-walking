/**
 * @file ContactsListProvider.cpp
 * @author Mohamed Elobaid
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#include <yarp/sig/Vector.h>

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <ContactsListProvider/ContactsListProvider.h>

using namespace StableCentroidalMPCWalking;

struct ContactsListProvider::Impl
{

    BipedalLocomotion::Contacts::ContactListMap m_contactListMap;

    UnicycleGenerator m_trajectoryGenerator;
    UnicycleController m_unicycleController;
    std::shared_ptr<FeetGenerator> m_feetGenerator;

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

    StepList m_leftSteps, m_rightSteps;

    double m_time;
    double m_plannerPeriod, m_plannerHorizon;

    iDynTree::Vector2 m_currentUnicyclePosition;
    iDynTree::Vector2 m_unicycleControlInput;

    yarp::os::BufferedPort<yarp::sig::Vector> m_inputPort;
    std::string m_joypadCommandPortName;
    iDynTree::Vector2 m_desiredTrajectoryPoint;

    bool configurePlanner(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
    {

        m_unicyclePlanner = std::make_shared<UnicyclePlanner>();

        auto handlerPtr = handler.lock();
        if (handlerPtr == nullptr)
        {
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::configurePlanner] The "
                                            "handler is not valid.");
            return false;
        }

        auto ptr = handlerPtr->getGroup("PLANNER_PARAMETERS").lock();

        if (ptr == nullptr)
        {
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::configurePlanner] Unable "
                                            "to get the planner parameters.");
            return false;
        }

        iDynTree::Vector2 refOffset, saturation;
        if (!ptr->getParameter("referencePosition", refOffset)
            || !ptr->getParameter("saturationFactors", saturation))
        {
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::configurePlanner] Unable "
                                            "to get the reference position or the saturation "
                                            "factors.");
            return false;
        }

        if (!ptr->getParameter("joypadCommandPortName", m_joypadCommandPortName))
        {
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::configurePlanner] Unable "
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
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::configurePlanner] Unable "
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
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::configurePlanner] Unable "
                                            "to configure the planner.");
            return false;
        }

        std::string controllerType;
        if (!ptr->getParameter("controllerType", controllerType))
        {
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::configurePlanner] Unable "
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
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::configurePlanner] "
                                            "Invalid "
                                            "controller type.");
            return false;
        }

        if (!m_unicyclePlanner->setUnicycleController(m_unicycleController))
        {
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::configurePlanner] Unable "
                                            "to set the controller type.");
            return false;
        }

        m_feetGenerator = m_trajectoryGenerator.addFeetMinimumJerkGenerator();

        // TODO: remove me to config file
        m_unicyclePlanner->addTerminalStep(true);
        m_unicyclePlanner->startWithLeft(true);

        m_leftFootPrint = std::make_shared<FootPrint>();
        m_rightFootPrint = std::make_shared<FootPrint>();

        m_currentUnicyclePosition.zero();
        m_unicycleControlInput.zero();

        m_time = 0.0;

        return true;
    }

    bool configure()
    {
        if (!m_inputPort.open("/contacts-list-provider/joypad-input"))
        {
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::configure] Unable to "
                                            "open the input port.");
            return false;
        }

        if (!yarp::os::Network::connect(m_joypadCommandPortName, m_inputPort.getName()))
        {
            BipedalLocomotion::log()->warn("[ContactsListProvider::Impl::configure] Unable to "
                                           "connect to the joypad command port.");
        }

        return true;
    }

    bool updatePlannerDesiredTrajectory()
    {
        yarp::sig::Vector* command = m_inputPort.read(false);
        if (command == nullptr)
        {
            BipedalLocomotion::log()->warn("[ContactsListProvider::Impl::"
                                           "updatePlannerDesiredTrajectory] "
                                           "Unable to read the joypad command.");
            return false;
        }

        m_desiredTrajectoryPoint[0] = (*command)[0];
        m_desiredTrajectoryPoint[1] = (*command)[1];

        if (m_unicycleController == UnicycleController::PERSON_FOLLOWING)
        {
            // first we add the current point to the planner and zero time
            auto time = 0.0;
            m_currentUnicyclePosition.zero();
            // m_unicyclePlanner->getPersonPosition(time, m_currentUnicyclePosition);
            m_unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(time,
                                                                        m_currentUnicyclePosition);

            time += m_plannerPeriod;
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

        auto initialTime = 0.0;
        auto finalTime = initialTime + m_plannerHorizon;
        if (!m_unicyclePlanner->computeNewSteps(m_leftFootPrint,
                                                m_rightFootPrint,
                                                initialTime,
                                                finalTime))
        {
            BipedalLocomotion::log()->error("[ContactsListProvider::Impl::computeNewSteps] Unable "
                                            "to compute new steps.");
            return false;
        }

        m_leftSteps = m_leftFootPrint->getSteps();
        m_rightSteps = m_rightFootPrint->getSteps();
        for (auto step : m_leftSteps)
            {
                BipedalLocomotion::log()->info("number of left steps: {}", m_leftSteps.size());
                BipedalLocomotion::log()->info("left step position: {}",
                step.position.toString()); BipedalLocomotion::log()->info("left step angle: {}",
                iDynTree::rad2deg(step.angle));
            }

            m_trajectoryGenerator.getFeetStandingPeriods(leftInContactPeriod, rightInContactPeriod);

        m_leftFootPrint->clearSteps();
        m_rightFootPrint->clearSteps();

        return true;
    }

    std::vector<BipedalLocomotion::Contacts::PlannedContact>
    getPlannedContactFromStep(const std::vector<bool>& isFootInContactVector, const StepList& steps)
    {
        std::vector<BipedalLocomotion::Contacts::PlannedContact> contacts;

        if (isFootInContactVector.size() != steps.size())
        {
            // Size mismatch, return empty vector or handle error as needed
            return contacts;
        }

        constexpr double secondsToNanoseconds = 1e9; // Conversion factor from seconds to
                                                     // nanoseconds

        for (size_t i = 0; i < steps.size(); ++i)
        {
            const auto& step = steps[i];
            const bool footInContact = isFootInContactVector[i];

            BipedalLocomotion::Contacts::PlannedContact contact;
            contact.name = step.footName;

            // Convert impact and lift times from seconds to nanoseconds
            contact.activationTime = std::chrono::nanoseconds(
                static_cast<long long>(step.impactTime * secondsToNanoseconds));
            contact.deactivationTime = footInContact
                                           ? std::chrono::nanoseconds::max()
                                           : std::chrono::nanoseconds(static_cast<long long>(
                                               step.impactTime * secondsToNanoseconds));

            contacts.push_back(contact);
        }

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

        return true;
    }

    ~Impl()
    {
        m_inputPort.close();
    }
};

ContactsListProvider::ContactsListProvider()
{
    m_pimpl = std::make_unique<Impl>();
}

ContactsListProvider::~ContactsListProvider() = default;

bool ContactsListProvider::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    if (!m_pimpl->configurePlanner(handler))
    {
        BipedalLocomotion::log()->error("[ContactsListProvider::initialize] Unable to configure "
                                        "the planner.");
        return false;
    }

    if (!m_pimpl->configure())
    {
        BipedalLocomotion::log()->error("[ContactsListProvider::initialize] Unable to configure "
                                        "the contacts list provider.");
        return false;
    }

    m_pimpl->m_generatorState = Impl::FSM::Initialized;

    return true;
}

bool ContactsListProvider::isOutputValid() const
{
    return m_pimpl->m_generatorState == Impl::FSM::OutputValid;
}

bool ContactsListProvider::advance()
{
    constexpr auto logPrefix = "[ContactsListProvider::advance]";

    if (!m_pimpl->updatePlannerDesiredTrajectory())
    {
        BipedalLocomotion::log()->warn("{} Unable to update the planner desired trajectory.",
                                       logPrefix);
    }

    if (!m_pimpl->computeSteps())
    {
        BipedalLocomotion::log()->warn("{} Unable to compute the steps.", logPrefix);
    }

    // get the contact list
    if (!m_pimpl->computeContactList())
    {
        BipedalLocomotion::log()->warn("{} Unable to compute the contact list.", logPrefix);
    }

    return true;
}

const BipedalLocomotion::Contacts::ContactListMap& ContactsListProvider::getOutput() const
{
    return m_pimpl->m_contactListMap;
}
