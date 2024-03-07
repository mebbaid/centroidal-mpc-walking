/**
 * @file ContactsListProvider.h
 * @author Mohamed Elobaid
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#ifndef CONTACTSLISTPROVIDER_H
#define CONTACTSLISTPROVIDER_H

#include <memory>

#include "iDynTree/Core/EigenHelpers.h"


#include <yarp/os/BufferedPort.h>
#include <UnicycleGenerator.h>
#include <FootPrint.h>


#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Contacts/ContactList.h>

namespace StableCentroidalMPCWalking
{


struct SimpleGeneratorOutput
{
    BipedalLocomotion::Contacts::ContactListMap contactList;
    std::vector<iDynTree::Vector2> dcmPositions;
    std::vector<iDynTree::Vector2> dcmVelocities;
} ;





/**
 * @brief This class is responsible for providing the contact list to the MPC.
 * given a joypad command, it uses the unicycle planner to generate the footsteps and
 * converts them to a contact list.
 */
class ContactsListProvider : public BipedalLocomotion::System::Source<SimpleGeneratorOutput>
{
public:

    typename ContactsListProvider::Output m_output;

    ContactsListProvider();
    ~ContactsListProvider();

    BipedalLocomotion::Contacts::ContactListMap output;

    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                        handler) override;

    bool advance()  override;

    const Output& getOutput() const  override;

    bool isOutputValid() const  override;

private:

    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};
}; // namespace StableCentroidalMPC

#endif // CONTACTSLISTPROVIDER_H
