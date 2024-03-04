/**
 * @file JoypadProvider.cpp
 * @author Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the BSD-3 clause
 * license.
 */

#include <BipedalLocomotion/System/SharedResource.h>
#include <FakeJoypad/JoypadProvider.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/RpcClient.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace StableCentroidalMPCWalking;

struct JoypadProvider::Impl
{
    yarp::os::BufferedPort<yarp::sig::Vector> port;
    yarp::os::RpcClient rpcClientPort;
    std::string rpcServerName;
    std::string rpcPortName;

    JoypadSignal joypad;

    ~Impl()
    {
        port.close();
        rpcClientPort.close();
    }
};

JoypadProvider::JoypadProvider()
{
    m_pimpl = std::make_unique<Impl>();
}

JoypadProvider::~JoypadProvider() = default;

bool JoypadProvider::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("[JoypadProvider::initialize] The handler is not valid.");
        return false;
    }

    std::string localPortName;
    if (!ptr->getParameter("local", localPortName))
    {
        BipedalLocomotion::log()->error("[JoypadProvider::initialize] Unable to find the 'local' "
                                        "parameter.");
        return false;
    }

    return m_pimpl->port.open(localPortName);

    // rpc  configuration
    if (!ptr->getParameter("rpcServerName", m_pimpl->rpcServerName))
    {
        BipedalLocomotion::log()->error("[JoypadProvider::initialize] Unable to find the "
                                        "'rpcServerName' "
                                        "parameter.");
        return false;
    }

    if (!ptr->getParameter("rpcClientName", m_pimpl->rpcPortName))
    {
        BipedalLocomotion::log()->error("[JoypadProvider::initialize] Unable to find the "
                                        "'rpcPortName' "
                                        "parameter.");
        return false;
    }

    m_pimpl->rpcClientPort.open(m_pimpl->rpcPortName);

    // connect to the rpc server
    if (!yarp::os::Network::connect(m_pimpl->rpcServerName, m_pimpl->rpcPortName))
    {
        BipedalLocomotion::log()->warn("[JoypadProvider::initialize] Unable to connect to the "
                                        "rpc server. Make sure it is available");
    }

    return true;
}

bool JoypadProvider::advance()
{
    constexpr bool writeStrict = false;

    auto& output = m_pimpl->port.prepare();
    output.resize(4);
    output[0] = m_pimpl->joypad.leftAnalogX;
    output[1] = m_pimpl->joypad.leftAnalogY;
    output[2] = m_pimpl->joypad.rightAnalogX;
    output[3] = m_pimpl->joypad.rightAnalogY;
    m_pimpl->port.write(writeStrict);

    yarp::os::Bottle cmd, reply;

    if (m_pimpl->joypad.select > 0)
    {
        cmd.addString("prepareRobot");
        m_pimpl->rpcClientPort.write(cmd, reply);
    }
    else if (m_pimpl->joypad.start > 0)
    {
        cmd.addString("startWalking");
        m_pimpl->rpcClientPort.write(cmd, reply);
    }


    return true;
}

bool JoypadProvider::setInput(const JoypadSignal& input)
{
    m_pimpl->joypad = input;
    return true;
}