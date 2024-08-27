using Godot;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

public partial class Serial: Node2D {
    private SerialPort port = new SerialPort();
    private List < PortC > fail = new List < PortC > ();
    private
    const string Eof = "Eof";
    [Signal]
    public delegate void data_receivedEventHandler(string data);

    public Thread thread;
    public bool reset;
    private bool Skip;
    public class PortC: IEquatable < PortC > {
        public string portName;
        public int time;
        public override bool Equals(object obj) {
            if (obj == null) return false;
            PortC objPo = obj as PortC;
            if (objPo == null) return false;
            else return Equals(objPo);
        }
        public override string ToString() {
            if (time == 0) {
                return portName + " a";
            } else {
                return portName + " b";
            }
        }
        public bool Equals(PortC other) {
            if (other == null) return false;
            return this.portName.Equals(other.portName);
        }

        public override int GetHashCode() {
            return 0;
        }
    }
    private TaskCompletionSource < bool > tscb = new TaskCompletionSource < bool > ();
    private TaskCompletionSource < bool > tscl = new TaskCompletionSource < bool > ();
    private TaskCompletionSource < string > tscs = new TaskCompletionSource < string > ();
    private string raw;

    public override void _Ready() {
        GD.Print("try");
        thread = new Thread(_process);
        thread.Start();
    }

    public async void _process() {
        while (true)
        {
            string[] ports = SerialPort.GetPortNames();
            foreach(string portName in ports) {
                if (Checkport(portName, 0)) {
                    continue;
                }

                try {
                    if(!port.IsOpen){
                        
                        port.PortName = portName;
                        port.BaudRate = 128000;
                        port.DataBits = 8;
                        port.StopBits = StopBits.One;
                        port.Encoding = Encoding.UTF8;
                    port.DataReceived += data;
                    port.Open();
                    }
                    bool hasData = await tscb.Task;
                    if (hasData) {
                        string data = tscs.Task.Result;
                        // EmitSignal( nameof(data_received), data);
                        this.CallDeferred("emit_signal", "data_received", data);
                        break;
                    }
                    await tscl.Task;
                    port.Close();
                } catch (Exception ex) {
                    GD.Print($"Error accessing {port.PortName}: {ex.Message} AT {ex.Source} {ex.StackTrace}");
                    Checkport(port.PortName, 1);
                }
            
        }
        }
    }

    private void data(object sender, SerialDataReceivedEventArgs e) {
        try {
            SerialPort _p = (SerialPort) sender;
            if (_p.IsOpen) {
                tscl.TrySetResult(false);
                string Data = _p.ReadTo(Eof);
                tscl.TrySetResult(true);
                tscs.TrySetResult(Data);
                tscb.TrySetResult(true);
            };
        } catch (Exception ex) {
            GD.Print($"Error accessing {port.PortName}: {ex.Message} AT {ex.Source} {ex.StackTrace}");
        }
    }


    private bool Checkport(string portName, int v) {
        if (v == 1) {
            if (fail.Exists(p => p.portName == portName)) {
                if (fail.Exists(p => p.time == 0)) {
                    int dex = fail.FindIndex(p => p.portName == portName);
                    fail.Add(new PortC() {
                        portName = fail[dex].portName, time = 1
                    });
                    fail.RemoveAt(dex);
                } else {
                    int dex = fail.FindIndex(p => p.portName == portName);
                    GD.PrintErr("!!! AT " + portName + "!!! " + "time:" + fail[dex].time + "dex: " + dex);
                }
            } else {
                fail.Add(new PortC() {
                    portName = portName, time = 0
                });
            }
            return false;
        } else {
            int dex = fail.FindIndex(p => p.portName == portName);
            if (dex == -1) {
                return false;
            }
            if (fail[dex].time == 0) {
                return false;
            } else {
                return true;
            }
        }
    }

}