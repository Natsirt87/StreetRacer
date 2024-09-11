using Godot;
using Interaction;
using Management;
using System;
using System.Collections.Generic;

namespace Networking;

public partial class NetworkManager : Node 
{
    private Dictionary<int, NetworkedController> _controllerList;

    [Export]
    public Node3D[] StartPositions;

    public override void _Ready()
    {
        //
        foreach (KeyValuePair<int, InfoPacket> entry in NetworkSession.Instance.Players)
        {

        }
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
        _controllerList[Multiplayer.GetRemoteSenderId()].ApplyPhysicsState();
    }

    [Rpc(MultiplayerApi.RpcMode.AnyPeer, TransferMode = MultiplayerPeer.TransferModeEnum.UnreliableOrdered, TransferChannel = 4)]
    public void ReceiveInputPacket()
    {
        _controllerList[Multiplayer.GetRemoteSenderId()].ApplyInputState();
    }
}