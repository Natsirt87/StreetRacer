using Godot;
using Interaction;
using Management;
using System;
using System.Collections.Generic;
using System.Net.Sockets;

namespace Networking;

public partial class NetworkSession : Node {
     
    [Signal]
    public delegate void PlayerConnectedEventHandler(int peerId, string playerName);
    [Signal]
    public delegate void PlayerDisconnectedEventHandler(int peerId);
    [Signal]
    public delegate void ServerDisconnectedEventHandler();

    public static NetworkSession Instance { get; private set; }

    public Dictionary<int, InfoPacket> Players;

    private InfoPacket _playerInfo;
    private int _playersLoaded;

    private const int Port = 6969;
    private const string DefaultServerIp = "127.0.0.1";
    private const int MaxConnections = 20;


    public override void _Ready()
    {
        Instance = this;

        Players = new Dictionary<int, InfoPacket>();
        _playerInfo = new InfoPacket();
        _playerInfo.Name = "Test Name";

        Multiplayer.PeerConnected += OnPlayerConnected;
        Multiplayer.PeerDisconnected += OnPlayerDisconnected;
        Multiplayer.ConnectedToServer += OnConnectedOk;
        Multiplayer.ConnectionFailed += OnConnectedFail;
        Multiplayer.ServerDisconnected += OnServerDisconnected;
    }

    public Error JoinGame(string address = "")
    {
        if (address.Equals(""))
            address = DefaultServerIp;
        
        ENetMultiplayerPeer peer = new ENetMultiplayerPeer();
        Error err = peer.CreateClient(address, Port);
        if (err != 0)
            return err;
        
        Multiplayer.MultiplayerPeer = peer;

        GD.Print("Joining game");

        return 0;
    }

    public Error CreateGame()
    {
        ENetMultiplayerPeer peer = new ENetMultiplayerPeer();
        Error err = peer.CreateServer(Port, MaxConnections);
        if (err != 0)
            return err;
        
        Multiplayer.MultiplayerPeer = peer;

        Players[1] = _playerInfo;
        EmitSignal(SignalName.PlayerConnected, 1, _playerInfo.Name);

        GD.Print("Creating game");

        return 0;
    }

    public void Disconnect()
    {
        Multiplayer.MultiplayerPeer.Close();
        Multiplayer.MultiplayerPeer = null;
        Players.Clear();

        GD.Print("Disconnected");
    }

    [Rpc(MultiplayerApi.RpcMode.AnyPeer, TransferMode = MultiplayerPeer.TransferModeEnum.Reliable, TransferChannel = 1)]
    private void RegisterPlayer(byte[] playerInfoBytes)
    {
        InfoPacket newPlayerInfo = PacketSerializer.ReadInfo(playerInfoBytes);
        int newPlayerId = Multiplayer.GetRemoteSenderId();
        Players.Add(newPlayerId, newPlayerInfo);

        GD.Print("Player registered: " + newPlayerInfo.Name);

        EmitSignal(SignalName.PlayerConnected, newPlayerId, newPlayerInfo.Name);
    }

    private void OnPlayerConnected(long id)
    {
        Rpc("RegisterPlayer", PacketSerializer.WriteInfo(_playerInfo));

        GD.Print("Player Connected");
    }

    private void OnPlayerDisconnected(long id)
    {
        Players.Remove((int)id);
        EmitSignal(SignalName.PlayerDisconnected, id);

        GD.Print("Player disconnected");
    }

    private void OnConnectedOk()
    {
        int peerId = Multiplayer.GetUniqueId();
        Players[peerId] = _playerInfo;
        EmitSignal(SignalName.PlayerConnected, peerId, _playerInfo.Name);

        GD.Print("Connected OK");
    }

    private void OnConnectedFail()
    {
        Multiplayer.MultiplayerPeer = null;
    }

    private void OnServerDisconnected()
    {
        Multiplayer.MultiplayerPeer = null;
        Players.Clear();
        EmitSignal(SignalName.ServerDisconnected);
    }
}