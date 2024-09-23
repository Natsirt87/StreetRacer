using Godot;
using System;
using System.IO;

namespace Management;

public struct PhysicsPacket
{
    public Vector3 Position;
    public Vector3 Rotation;

    public Vector3 Accel;
    public Vector3 Ang_accel;

    public Vector3 Jerk;
    public Vector3 Ang_jerk;
}

public struct InputPacket 
{
    public float Throttle;
    public float Brake;
    public float Steering;

    public float Rpm;
    public int Gear;
}

public struct InfoPacket
{
    public string Name;
    public string Car;
}

public static class PacketSerializer
{
    public static byte[] WritePhysics(PhysicsPacket packet)
    {
        MemoryStream stream = new();
        BinaryWriter writer = new(stream);

        writer.Write(packet.Position.X);
        writer.Write(packet.Position.Y);
        writer.Write(packet.Position.Z);
        writer.Write(packet.Rotation.X);
        writer.Write(packet.Rotation.Y);
        writer.Write(packet.Rotation.Z);
        writer.Write(packet.Accel.X);
        writer.Write(packet.Accel.Y);
        writer.Write(packet.Accel.Z);
        writer.Write(packet.Ang_accel.X);
        writer.Write(packet.Ang_accel.Y);
        writer.Write(packet.Ang_accel.Z);
        writer.Write(packet.Jerk.X);
        writer.Write(packet.Jerk.Y);
        writer.Write(packet.Jerk.Z);
        writer.Write(packet.Ang_jerk.X);
        writer.Write(packet.Ang_jerk.Y);
        writer.Write(packet.Ang_jerk.Z);

        byte[] bytes = stream.ToArray();
        writer.Close();
        return bytes;
    }

    public static PhysicsPacket ReadPhysics(byte[] bytes)
    {
        BinaryReader reader = new(new MemoryStream(bytes));
        PhysicsPacket packet = new();
        
        packet.Position.X = reader.ReadSingle();
        packet.Position.Y = reader.ReadSingle();
        packet.Position.Z = reader.ReadSingle();
        packet.Rotation.X = reader.ReadSingle();
        packet.Rotation.Y = reader.ReadSingle();
        packet.Rotation.Z = reader.ReadSingle();
        packet.Accel.X = reader.ReadSingle();
        packet.Accel.Y = reader.ReadSingle();
        packet.Accel.Z = reader.ReadSingle();
        packet.Ang_accel.X = reader.ReadSingle();
        packet.Ang_accel.Y = reader.ReadSingle();
        packet.Ang_accel.Z = reader.ReadSingle();
        packet.Jerk.X = reader.ReadSingle();
        packet.Jerk.Y = reader.ReadSingle();
        packet.Jerk.Z = reader.ReadSingle();
        packet.Ang_jerk.X = reader.ReadSingle();
        packet.Ang_jerk.Y = reader.ReadSingle();
        packet.Ang_jerk.Z = reader.ReadSingle();

        reader.Close();

        return packet;
    }

    public static byte[] WriteInput(InputPacket packet)
    {
        MemoryStream stream = new();
        BinaryWriter writer = new(stream);

        writer.Write(packet.Throttle);
        writer.Write(packet.Brake);
        writer.Write(packet.Steering);
        writer.Write(packet.Rpm);
        writer.Write(packet.Gear);

        byte[] bytes = stream.ToArray();
        writer.Close();
        return bytes;
    }

    public static InputPacket ReadInput(byte[] bytes)
    {
        BinaryReader reader = new(new MemoryStream(bytes));
        InputPacket packet = new();
        
        packet.Throttle = reader.ReadSingle();
        packet.Brake = reader.ReadSingle();
        packet.Steering = reader.ReadSingle();
        packet.Rpm = reader.ReadSingle();
        packet.Gear = reader.ReadInt32();
        
        reader.Close();

        return packet;
    }

    public static byte[] WriteInfo(InfoPacket packet)
    {
        MemoryStream stream = new();
        BinaryWriter writer = new(stream);

        writer.Write(packet.Name);
        writer.Write(packet.Car);

        byte[] bytes = stream.ToArray();
        writer.Close();
        return bytes;
    }

    public static InfoPacket ReadInfo(byte[] bytes)
    {
        BinaryReader reader = new(new MemoryStream(bytes));
        InfoPacket packet = new();
        
        packet.Name = reader.ReadString();
        packet.Car = reader.ReadString();

        reader.Close();

        return packet;
    }

}