using Godot;
using Networking;
using System;

public partial class MultiplayerMenu : Control
{
    [Export]
    public TextEdit NameEdit;
    [Export]
    public Button CreateButton;
    [Export]
    public Button JoinButton;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
        NetworkSession session = NetworkSession.Instance;

        NameEdit.Text = session.PlayerInfo.Name;
        NameEdit.TextChanged += UpdateName;

        
        CreateButton.Pressed += () => {
            if (session.CreateGame() == 0) {
                GetTree().ChangeSceneToFile("res://scenes/ui/Lobby.tscn");
            }
        };

        JoinButton.Pressed += () => {
            if (session.JoinGame() == 0) {
                GetTree().ChangeSceneToFile("res://scenes/ui/Lobby.tscn");
            }
        };
	}

    private void UpdateName()
    {
        NetworkSession.Instance.PlayerInfo.Name = NameEdit.Text;
    }
}
