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
    public InfoPacket PlayerInfo;
    public int[] StartPositions;
    public int NetworkTickInterval = 60;

    private int _playersLoaded;
    
    private const int MaxConnections = 12;
    private const int Port = 7000;
    private const string DefaultServerIp = "127.0.0.1";

    public override void _Ready()
    {
        Instance = this;
        Multiplayer.MultiplayerPeer = null;

        StartPositions = new int[MaxConnections];

        Players = new Dictionary<int, InfoPacket>();
        PlayerInfo = new InfoPacket
        {
            Name = "Test Name",
            Car = "Skyline"
        };

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
        
        ENetMultiplayerPeer peer = new();
        Error err = peer.CreateClient(address, Port);
        if (err != 0)
            return err;
        
        Multiplayer.MultiplayerPeer = peer;

        GD.Print("Joining game");

        return 0;
    }

    public Error CreateGame()
    {
        ENetMultiplayerPeer peer = new();
        Error err = peer.CreateServer(Port, MaxConnections);
        if (err != 0)
            return err;
        
        Multiplayer.MultiplayerPeer = peer;

        Players[1] = PlayerInfo;
        EmitSignal(SignalName.PlayerConnected, 1, PlayerInfo.Name);

        StartPositions[0] = 1;

        GD.Print("Creating game");

        return 0;
    }

    public void TerminateConnection()
    {
        Multiplayer.MultiplayerPeer?.Close();
        Multiplayer.MultiplayerPeer = null;
        Players.Clear();
        StartPositions = new int[MaxConnections];
    }

    public void StartGame()
    {
        GD.Print("FROM SERVER -- Starting game");

        Rpc("SetStartPositions", StartPositions);
        Rpc("LoadGame", "res://scenes/tracks/TestTrack.tscn");
    }

    [Rpc(MultiplayerApi.RpcMode.AnyPeer, TransferMode = MultiplayerPeer.TransferModeEnum.Reliable, TransferChannel = 1)]
    private void RegisterPlayer(byte[] playerInfoBytes)
    {
        InfoPacket newPlayerInfo = PacketSerializer.ReadInfo(playerInfoBytes);
        int newPlayerId = Multiplayer.GetRemoteSenderId();
        Players.Add(newPlayerId, newPlayerInfo);

        if (Multiplayer.IsServer())
        {
            StartPositions[Players.Count - 1] = newPlayerId;
            GD.Print("FROM SERVER -- Start Positions Updated: " + newPlayerId + " is in position " + Players.Count);
        }

        GD.Print("FROM [" + Multiplayer.GetUniqueId() + "] -- Player Registered: " + newPlayerInfo.Name);

        EmitSignal(SignalName.PlayerConnected, newPlayerId, newPlayerInfo.Name);
    }

    [Rpc(MultiplayerApi.RpcMode.Authority, TransferMode = MultiplayerPeer.TransferModeEnum.Reliable, TransferChannel = 1)]
    private void SetStartPositions(int[] startPositions)
    {
        StartPositions = startPositions;

        GD.Print("New start positions:");

        for (int i = 0; i < MaxConnections; i++)
        {
            GD.Print(StartPositions[i] + " is at position " + (i + 1));
        }
    }

    [Rpc(MultiplayerApi.RpcMode.Authority, TransferMode = MultiplayerPeer.TransferModeEnum.Reliable, TransferChannel = 1, CallLocal = true)]
    private void LoadGame(String scenePath)
    {
        GetTree().ChangeSceneToFile(scenePath);
    }

    private void OnPlayerConnected(long id)
    {
        RpcId(id, "RegisterPlayer", PacketSerializer.WriteInfo(PlayerInfo));

        GD.Print("FROM [" + Multiplayer.GetUniqueId() + "] -- Player Connected: " + id);
    }

    private void OnPlayerDisconnected(long id)
    {
        Players.Remove((int)id);
        EmitSignal(SignalName.PlayerDisconnected, id);

        GD.Print("FROM [" + Multiplayer.GetUniqueId() + "] -- Player Disconnected: " + id);
    }

    private void OnConnectedOk()
    {
        int peerId = Multiplayer.GetUniqueId();
        Players[peerId] = PlayerInfo;
        EmitSignal(SignalName.PlayerConnected, peerId, PlayerInfo.Name);

        GD.Print("Connected OK");
    }

    private void OnConnectedFail()
    {
        TerminateConnection();
    }

    private void OnServerDisconnected()
    {
        TerminateConnection();
        EmitSignal(SignalName.ServerDisconnected);

        GD.Print("Server disconnected");
    }
}