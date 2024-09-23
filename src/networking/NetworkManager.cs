using Godot;
using Interaction;
using Management;
using System;
using System.Collections.Generic;
using VehiclePhysics;

namespace Networking;

public partial class NetworkManager : Node 
{
    [Export]
    public GameManager Game;

    public static NetworkManager Instance { get; private set; }

    public override void _Ready()
    {
        Instance = this;
    }

    public void SendPhysicsPacket(PhysicsPacket physicsData)
    {
        
    }

    public void SendInputPacket(InputPacket inputData)
    {

    }

    [Rpc(MultiplayerApi.RpcMode.AnyPeer, TransferMode = MultiplayerPeer.TransferModeEnum.UnreliableOrdered, TransferChannel = 2)]
    public void ReceivePhysicsPacket(byte[] physicsDataBytes)
    {
        PhysicsPacket physicsData = PacketSerializer.ReadPhysics(physicsDataBytes);
        Game.NetworkedControllers[Multiplayer.GetRemoteSenderId()].ApplyPhysicsState(physicsData);
    }

    [Rpc(MultiplayerApi.RpcMode.AnyPeer, TransferMode = MultiplayerPeer.TransferModeEnum.UnreliableOrdered, TransferChannel = 4)]
    public void ReceiveInputPacket(byte[] inputDataBytes)
    {
        InputPacket inputData = PacketSerializer.ReadInput(inputDataBytes);
        Game.NetworkedControllers[Multiplayer.GetRemoteSenderId()].ApplyInputState(inputData);
    }
}