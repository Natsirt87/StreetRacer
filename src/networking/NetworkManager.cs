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

    public override void _Ready()
    {
        
    }

    public void SendPhysicsPacket()
    {
        
    }

    public void SendInputPacket()
    {

    }

    [Rpc(MultiplayerApi.RpcMode.AnyPeer, TransferMode = MultiplayerPeer.TransferModeEnum.UnreliableOrdered, TransferChannel = 2)]
    public void ReceivePhysicsPacket()
    {
        Game.NetworkedControllers[Multiplayer.GetRemoteSenderId()].ApplyPhysicsState();
    }

    [Rpc(MultiplayerApi.RpcMode.AnyPeer, TransferMode = MultiplayerPeer.TransferModeEnum.UnreliableOrdered, TransferChannel = 4)]
    public void ReceiveInputPacket()
    {
        Game.NetworkedControllers[Multiplayer.GetRemoteSenderId()].ApplyInputState();
    }
}