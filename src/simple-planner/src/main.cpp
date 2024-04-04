/**
 * @file main.cpp
 * @author Mohamed Elobaid
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#include <cstdlib>
#include <memory>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/AdvanceableRunner.h>
#include <BipedalLocomotion/System/Barrier.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/QuitHandler.h>
#include <BipedalLocomotion/System/SharedResource.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <SimplePlanner/SimplePlanner.h>

int main(int argc, char* argv[])
{
    constexpr auto errorPrefix = "[main]";

    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // initialize yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        BipedalLocomotion::log()->error("[main] Unable to find YARP network.");
        return EXIT_FAILURE;
    }

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("planner.ini");
    rf.configure(argc, argv);

    auto handler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    handler->set(rf);

    // create the contact list provider
    auto contactListProvider = std::make_unique<StableCentroidalMPCWalking::SimplePlanner>();
    if (!contactListProvider->initialize(handler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the contact list provider.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    auto input0 = BipedalLocomotion::System::SharedResource<
        StableCentroidalMPCWalking::SimplePlanner::Input>::create();

    auto output0 = BipedalLocomotion::System::SharedResource<
        StableCentroidalMPCWalking::SimplePlanner::Output>::create();
    // create the advanceable runner
    BipedalLocomotion::System::AdvanceableRunner<StableCentroidalMPCWalking::SimplePlanner> contactListProviderRunner;

    if (!contactListProviderRunner.initialize(handler->getGroup("CONTACTSLIST_PROVIDER_RUNNER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the contact list provider runner.",
                                        errorPrefix);
        return EXIT_FAILURE;
    }

    contactListProviderRunner.setInputResource(input0);
    contactListProviderRunner.setOutputResource(output0);
    contactListProviderRunner.setAdvanceable(std::move(contactListProvider));

    //handle quit signal
    BipedalLocomotion::System::handleQuitSignals([&]() {contactListProviderRunner.stop();});

    // run thread
    auto barrier = BipedalLocomotion::System::Barrier::create(1);
    auto threadSimplePlanner = contactListProviderRunner.run(barrier);

    while (contactListProviderRunner.isRunning())
    {
        using namespace std::chrono_literals;
        constexpr auto delay = 100ms;

        // release the CPU
        BipedalLocomotion::clock().yield();
        BipedalLocomotion::clock().sleepFor(delay);
    }

    contactListProviderRunner.stop();

    if (threadSimplePlanner.joinable())
    {
        threadSimplePlanner.join();
    }

    return EXIT_SUCCESS;
}
