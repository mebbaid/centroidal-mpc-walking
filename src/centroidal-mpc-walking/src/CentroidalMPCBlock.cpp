/**
 * @file CentroidalMPCBlock.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Contacts/ContactListJsonParser.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>

#include <CentroidalMPCWalking/CentroidalMPCBlock.h>
#include <manif/manif.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace CentroidalMPCWalking;
using namespace BipedalLocomotion::ParametersHandler;

bool isFirstRun{true};
Eigen::MatrixXd comTraj(3, 1500);
int indexCoM{0};
bool CentroidalMPCBlock::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    if (!m_controller.initialize(handler))
    {
        BipedalLocomotion::log()->error("[CentroidalMPCBlock::initialize] Unable to initialize the "
                                        "controller");
        return false;
    }

    return true;
}

const CentroidalMPCBlock::Output& CentroidalMPCBlock::getOutput() const
{
    return m_controller.getOutput();
}

bool CentroidalMPCBlock::setInput(const Input& input)
{
    if (input.isValid)
    {
        m_inputValid = input.isValid;

        if (isFirstRun)
        {
            isFirstRun = false;

            // TODO Remove me!!!
            // left foot
            // first footstep

            constexpr double scaling = 1;
            constexpr double scalingPos = 1.5;
            constexpr double scalingPosY = 0;
            // // t  0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19
            // 20  21  22  23  24  25  26  27
            // // L
            // |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|++++++++++|---|+++|
            // // R
            // |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++|
            BipedalLocomotion::Contacts::ContactListMap contactListMap;

            Eigen::Vector3d leftPosition = input.leftFoot.translation();
            manif::SE3d leftTransform(leftPosition, manif::SO3d::Identity());
            contactListMap["left_foot"].addContact(leftTransform, 0.0, 1.0 * scaling);

            leftPosition(0) += 0.05 * scalingPos;
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 2.0 * scaling, 5.0 * scaling);

            leftPosition(0) += 0.1 * scalingPos;
            leftPosition(2) = 0.0 + 0.0;
            // leftTransform.quat(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 6.0 * scaling, 9.0 * scaling);

            leftPosition(0) += 0.1 * scalingPos;
            leftPosition(2) = 0.0;
            leftTransform.quat(manif::SO3d::Identity());
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 10.0 * scaling, 13.0 * scaling);

            leftPosition(0) += 0.1 * scalingPos;
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 14.0 * scaling, 17.0 * scaling);

            leftPosition(1) -= 0.01 * scalingPosY;
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 18.0 * scaling, 21.0 * scaling);

            leftPosition(1) -= 0.01 * scalingPosY;
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 22.0 * scaling, 25.0 * scaling);

            leftPosition(1) -= 0.01 * scalingPosY;
            leftTransform.translation(leftPosition);
            contactListMap["left_foot"].addContact(leftTransform, 26.0 * scaling, 29.0 * scaling);

            // // t  0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19
            // 20  21  22  23  24  25  26  27
            // // L
            // |+++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|++++++++++|---|+++|
            // // R
            // |+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++++|---|+++++++++|

            // right foot
            // first footstep
            Eigen::Vector3d rightPosition = input.rightFoot.translation();
            manif::SE3d rightTransform(rightPosition, manif::SO3d::Identity());

            contactListMap["right_foot"].addContact(rightTransform, 0.0, 3.0 * scaling);

            rightPosition(0) += 0.1 * scalingPos;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 4.0 * scaling, 7.0 * scaling);

            rightPosition(0) += 0.1 * scalingPos;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 8.0 * scaling, 11.0 * scaling);

            rightPosition(0) += 0.1 * scalingPos;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 12.0 * scaling, 15.0 * scaling);

            rightPosition(0) += 0.0 * scalingPos;
            rightPosition(1) -= 0.01 * scalingPosY;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 16.0 * scaling, 19.0 * scaling);

            rightPosition(1) -= 0.01 * scalingPosY;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 20.0 * scaling, 23.0 * scaling);

            rightPosition(1) -= 0.01 * scalingPosY;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 24.0 * scaling, 27.0 * scaling);

            rightPosition(1) -= 0.01 * scalingPosY;
            rightTransform.translation(rightPosition);
            contactListMap["right_foot"].addContact(rightTransform, 28.0 * scaling, 29.0 * scaling);

            // contactListMap =
            // BipedalLocomotion::Contacts::contactListMapFromJson("footsteps.json");
            m_phaseList.setLists(contactListMap);

            std::vector<Eigen::VectorXd> comKnots;
            std::vector<double> timeKnots;

            BipedalLocomotion::Planners::QuinticSpline comSpline;
            timeKnots.push_back(m_phaseList.cbegin()->beginTime);
            comKnots.push_back(input.com);
            for (auto it = m_phaseList.begin(); it != m_phaseList.end(); std::advance(it, 1))
            {
                if (it->activeContacts.size() == 2 && it != m_phaseList.begin()
                    && it != m_phaseList.lastPhase())
                {
                    timeKnots.emplace_back((it->endTime + it->beginTime) / 2);

                    auto contactIt = it->activeContacts.cbegin();
                    const Eigen::Vector3d p1 = contactIt->second->pose.translation();
                    std::advance(contactIt, 1);
                    const Eigen::Vector3d p2 = contactIt->second->pose.translation();

                    Eigen::Vector3d desiredCoMPosition = (p1 + p2) / 2.0;
                    desiredCoMPosition(2) += input.com(2);

                    comKnots.emplace_back(desiredCoMPosition);
                }

                else if (it->activeContacts.size() == 2 && it == m_phaseList.lastPhase())
                {
                    timeKnots.push_back(it->endTime);
                    auto contactIt = it->activeContacts.cbegin();
                    const Eigen::Vector3d p1 = contactIt->second->pose.translation();
                    std::advance(contactIt, 1);
                    const Eigen::Vector3d p2 = contactIt->second->pose.translation();

                    Eigen::Vector3d desiredCoMPosition = (p1 + p2) / 2.0;
                    desiredCoMPosition(2) += input.com(2);

                    comKnots.emplace_back(desiredCoMPosition);
                }
            }

            comSpline.setInitialConditions(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            comSpline.setFinalConditions(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            comSpline.setKnots(comKnots, timeKnots);

            Eigen::Vector3d velocity, acceleration;

            int tempInt = 1000;
            for (int i = 0; i < tempInt / scaling; i++)
            {
                // TODO remove me
                comSpline.evaluatePoint(i * 0.1, comTraj.col(i), velocity, acceleration);
            }

            // i = 3
            // * *
            // 1 2 3 4 5
            comTraj.rightCols(comTraj.cols() - tempInt).colwise() = comTraj.col(tempInt - 1);
        }

        return m_controller.setState(input.com, input.dcom, input.angularMomentum);
    }
    return true;
}

bool CentroidalMPCBlock::advance()
{
    namespace blf = ::BipedalLocomotion;
    constexpr auto logPrefix = "[CentroidalMPCBlock::advance]";

    if (m_inputValid)
    {
        if (!m_controller.setReferenceTrajectory(comTraj.rightCols(comTraj.cols() - indexCoM)))
        {
            blf::log()->error("{} Unable to set the reference CoM position.", logPrefix);
            return false;
        }

        if (!m_controller.setContactPhaseList(m_phaseList))
        {
            blf::log()->error("{} Unable to set the contact phase list.", logPrefix);
            return false;
        }

        if (!m_controller.advance())
        {
            blf::log()->error("{} Unable to compute the control output.", logPrefix);
            return false;
        }

        indexCoM++;
        return true;
    }
    return true;
}

bool CentroidalMPCBlock::isOutputValid() const
{
    return m_controller.isOutputValid();
}
