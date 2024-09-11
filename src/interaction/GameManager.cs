using Godot;
using Networking;
using System;
using System.Collections.Generic;

namespace Interaction;

public partial class GameManager : Node3D
{
    [Export(PropertyHint.File)]
    public string PlayerVehiclePath;
    [Export(PropertyHint.File)]
    public string PlayerCameraPath;
    [Export]
    public Node3D[] StartNodes;

    public Dictionary<int, NetworkedController> NetworkedControllers;
    // Future AI controller management list: public List<AIController> AIControllers;
    public PlayerController Player;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
        if (Multiplayer.MultiplayerPeer == null) {
            SpawnPlayerController(0, false);
            GD.Print("Singleplayer mode");
            return;
        }

        NetworkedControllers = new();
        NetworkSession session = NetworkSession.Instance;

        int playerStartPos = Array.IndexOf(session.StartPositions, Multiplayer.GetUniqueId());
        GD.Print("Spawning player " + Multiplayer.GetUniqueId() + " at position " + playerStartPos);
        SpawnPlayerController(playerStartPos, true);

        SpawnNetworkedControllers(session);
	}

    private void SpawnPlayerController(int startPos, bool networked)
    {
        Player = new(PlayerVehiclePath, PlayerCameraPath, networked);
        StartNodes[startPos].AddChild(Player);
    }

    private void SpawnNetworkedControllers(NetworkSession session) 
    {
        for (int i = 0; i < session.Players.Count; i++)
        {
            int peerId = session.StartPositions[i];

            if (peerId == Multiplayer.GetUniqueId())
                continue;
            
            NetworkedController controller = new(peerId);
            NetworkedControllers.Add(peerId, controller);
            StartNodes[i].AddChild(controller);

            GD.Print("Spawning networked player " + peerId + " at position " + i);
        }
    }

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}
}
