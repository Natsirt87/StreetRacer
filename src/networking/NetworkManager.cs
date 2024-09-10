using Godot;
using Interaction;
using System;
using System.Collections.Generic;

namespace Networking;

public partial class NetworkManager : Node 
{

    private Dictionary<int, NetworkedController> _playerList;

    public override void _Ready()
    {
        foreach (Node playerNode in GetChildren())
        {
            NetworkedController playerController = (NetworkedController) playerNode;
            _playerList.Add(playerController.PeerId, playerController);
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
        _playerList[Multiplayer.GetRemoteSenderId()].ApplyPhysicsState();
    }

    [Rpc(MultiplayerApi.RpcMode.AnyPeer, TransferMode = MultiplayerPeer.TransferModeEnum.UnreliableOrdered, TransferChannel = 4)]
    public void ReceiveInputPacket()
    {
        _playerList[Multiplayer.GetRemoteSenderId()].ApplyInputState();
    }
}