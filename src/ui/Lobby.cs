using Godot;
using Management;
using Networking;
using System;
using System.Collections.Generic;
using System.Diagnostics.Metrics;

namespace UI;

public partial class Lobby : Control
{
    [Export]
    public Button StartButton;
    [Export]
    public Button DisconnectButton;
    [Export]
    public ItemList PlayerList;

    private Dictionary<int, int> _idToIdx;
    
	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
        _idToIdx = new Dictionary<int, int>();

        StartButton.Disabled = !Multiplayer.IsServer();

        StartButton.Pressed += NetworkSession.Instance.StartGame;
        DisconnectButton.Pressed += Disconnect;
        NetworkSession.Instance.ServerDisconnected += OnServerDisconnect;
        NetworkSession.Instance.PlayerConnected += AddToPlayerList;
        NetworkSession.Instance.PlayerDisconnected += RemoveFromPlayerList;

        foreach (KeyValuePair<int, InfoPacket> entry in NetworkSession.Instance.Players)
        {
            AddToPlayerList(entry.Key, entry.Value.Name);
        }

        GD.Print("Loaded lobby");
	}

    private void OnServerDisconnect()
    {
        GetTree().ChangeSceneToFile("res://scenes/ui/MultiplayerMenu.tscn");
    }

    private void Disconnect()
    {
        NetworkSession.Instance.TerminateConnection();
        GetTree().ChangeSceneToFile("res://scenes/ui/MultiplayerMenu.tscn");
    }

    private void AddToPlayerList(int peerId, string playerName)
    {
        int idx = PlayerList.AddItem(playerName);
        _idToIdx[peerId] = idx;
    }

    private void RemoveFromPlayerList(int peerId)
    {
        int idx = _idToIdx[peerId];
        PlayerList.RemoveItem(idx);
        _idToIdx.Remove(peerId);
    }

    public override void _ExitTree()
    {
        DisconnectButton.Pressed -= Disconnect;
        NetworkSession.Instance.ServerDisconnected -= OnServerDisconnect;
        NetworkSession.Instance.PlayerConnected -= AddToPlayerList;
        NetworkSession.Instance.PlayerDisconnected -= RemoveFromPlayerList;
    }
}
