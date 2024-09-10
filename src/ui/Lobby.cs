using Godot;
using Networking;
using System;

namespace UI;

public partial class Lobby : Control
{
    [Export]
    public Button CreateButton;
    [Export]
    public Button JoinButton;
    [Export]
    public Button DisconnectButton;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
        NetworkSession session = NetworkSession.Instance;
        CreateButton.Pressed += () => session.CreateGame();
        JoinButton.Pressed += () => session.JoinGame();
        DisconnectButton.Pressed += () => session.Disconnect();
	}
}
