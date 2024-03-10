using SciChart.Charting.Model.DataSeries;
using SciChart.Charting.Visuals.Axes;
using SciChart.Examples.ExternalDependencies.Data;
using System;
using System.Net.Sockets;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;
using System.Data;
using System.Threading;
using SciChart.Data.Model;
using System.Windows.Media;
using System.Runtime.Remoting.Contexts;
using System.Windows.Markup;
using System.IO;
//using OpenFileDialog = System.Windows.Forms.OpenFileDialog;
using Microsoft.Win32;
using System.IO.MemoryMappedFiles;
using HidLibrary;
using System.Linq;

namespace SciChartExport
{
    /// <summary>
    /// Interaction logic for Shell.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region define
        private string[] axisAlignment = new[] { "Bottom", "Top", "Left", "Right" };

        UdpClient Lidar_Client = new UdpClient(1024);                                     // Port tren pc2 nhan du lieu tu pc1 den
        IPEndPoint Lidar_Clienti = new IPEndPoint(IPAddress.Parse("192.168.1.11"), 1025);    // Khai bao dia chi ip + port nhan cua pc1 
        IPEndPoint Lidar_Cliento = new IPEndPoint(IPAddress.Parse("192.168.1.11"), 1025);   // Khai bao dia chi ip + port nhan cua pc1 

        UdpClient Robo_Client = new UdpClient(1028);                                     // Port tren pc2 nhan du lieu tu pc1 den
        IPEndPoint Robo_Clienti = new IPEndPoint(IPAddress.Parse("192.168.1.12"), 1029);    // Khai bao dia chi ip + port nhan cua pc1 
        IPEndPoint Robo_Cliento = new IPEndPoint(IPAddress.Parse("192.168.1.12"), 1029);   // Khai bao dia c

        UdpClient Compass_Client = new UdpClient(1030);                                     // Port tren pc2 nhan du lieu tu pc1 den
        IPEndPoint Compass_Clienti = new IPEndPoint(IPAddress.Parse("192.168.1.13"), 1031);    // Khai bao dia chi ip + port nhan cua pc1 
        IPEndPoint Compass_Cliento = new IPEndPoint(IPAddress.Parse("192.168.1.13"), 1031);   // Khai bao dia c

        UdpClient Rabit_Client = new UdpClient(1032);                                     // Port tren pc2 nhan du lieu tu pc1 den
        IPEndPoint Rabit_Clienti = new IPEndPoint(IPAddress.Parse("192.168.1.14"), 1032);    // Khai bao dia chi ip + port nhan cua pc1 
        IPEndPoint Rabit_Cliento = new IPEndPoint(IPAddress.Parse("192.168.1.14"), 1032);   // Khai bao dia c
     
        DispatcherTimer Playback_time = new DispatcherTimer();
        OpenFileDialog openFileDialog1 = new OpenFileDialog();
        DispatcherTimer Check_USB = new DispatcherTimer();
        DispatcherTimer Check_Counter = new DispatcherTimer();

        Robot_Analyzer_def Robot_Analyzers;

        //Thread Thread_ethenet;
        Thread Thread_ethenet2;
        //Thread Thread_ethenet3;
        Thread Thread_ethenet4;
        //Thread Thread_ethenet5;
        Thread Thread_USB;

        PID_struct_def PID_struct;
        HidReport HID_report;
        private static HidDevice device;

        #endregion

        #region struct
        struct Robot_Analyzer_def
        {
            public double[] _re1;
            public double[] _im1;

            // vị trí di chuyển
            public double[] _re_R1;
            public double[] _im_R1;

            // duty1
            public double[] _re_R2;
            public double[] _im_R2;

            // duty2
            public double[] _re_R3;
            public double[] _im_R3;

            // duty3
            public double[] _re_R4;
            public double[] _im_R4;

            // duty4
            public double[] _re_R5;
            public double[] _im_R5;

            // ADC1
            public double[] _re_R6;
            public double[] _im_R6;

            // ADC2
            public double[] _re_R7;
            public double[] _im_R7;

            public UInt16 connect;
            public UInt16 Counters;
            public UInt16 Counters2;

            public UInt32 Max_ADC;
            public Int32 Max_Compass;
            public Int32 Min_Compass;
            public UInt32 Max_ABC;

            public UInt32 Max_ADCi;
            public Int32 Max_Compassi;
            public Int32 Min_Compassi;
            public UInt32 Max_ABCi;

            public byte[] Lidar_Frame;
            public byte[] Robo_Frame;
            public byte[] Robo_Frame2;
            public byte[] Compass_Frame;

            public int[] Robo_duty;
            public uint[] Robo_dir;
            public int Robo_Encoder;
            public double Robo_Compass;
            public uint Robo_ADC1;
            public uint Robo_ADC2;
            public uint Robo_ADC3;
            public uint Robo_A_Val;
            public uint Robo_B_Val;
            public uint Robo_C_Val;
            public uint Robo_Input_Val;
            public uint Robo_Input_Val2;
            public uint Robo_Input_Val3;
            public uint[] Robo_bit;
            public uint[] Robo_bit2;
            public uint[] Robo_bit3;
            public uint[] Robo_bit4;
            public UInt16 Robo_Sample;
            public UInt16 Robo_Restart;
            public UInt16 Robo_Restart2;
            public byte[] Robo_PES;

            public int Compass_R1;
            public int Compass_R2;
            public int Compass_R3;
            public int Compass_R4;
            public double Compass_A1;
            public double Compass_A2;
            public double Compass_A3;
            public double Compass_A4;
            public double Compass_A5;
            public int Compass_Offset1;
            public int Compass_Offset2;
            public int Compass_Offset3;
            public int Compass_Offset4;

            public int Compass_Raw_Max;
            public int Compass_Raw_Min;

            public double Compass_Result_Max;
            public double Compass_Result_Min;

            public int Compass_Raw_Maxi;
            public int Compass_Raw_Mini;

            public double Compass_Result_Maxi;
            public double Compass_Result_Mini;

            public double Compass_input3_Max;
            public double Compass_input3_Maxi;

            public uint Robot_Point;
            public uint Robot_Running;

            public UInt16 Compass_Sample;

            public int Clear_Request1;
            public int Clear_Request2;

            public UInt16[] Robot_EEpromi;
            public UInt16[] Robot_EEprom;
            public int EEprom_Mode;
            public int EEprom_Save;
            public int EEprom_Load;
            public int EEprom_Read_Point;
            public int EEprom_Save_file;
            public int EEprom_Load_file;

            public byte[] Robot_Packet_Frame;
            public int Robot_Packet_Count;
            public int Robot_Packet_View;
            public int Robot_Packet_Step;
            public byte[] Robot_Packet_USB;
            public int Robot_Packet_Save;
            public int Robot_Packet_Save_File;
            public int Robot_Thread_Run;
            public int Robot_Analyse_Start;

            public int Robot_Analyse;
            public int Robot_View_Mode;
            public int Robot_View_Stop;
            public int Robot_View_Stop2;
            public int Robot_View_SYNC;
            public int Robot_View_Clear2;
            public int Robot_View_Cleari2;
            public int Robot_View_Clear;
            public int Robot_View_Cleari;
            public int Robot_View_Delay;

            public int Robot_Counter;
            public int Robot_Clear;
            public int Robot_View1;
            public int Robot_View2;
            public int Robot_View3;
            public int Robot_View4;

            public int Robot_GPIO1;
            public int Robot_GPIO2;
            public int Robot_GPIO3;
            public int Robot_GPIO4;
            public int Robot_GPIO5;
            public int Robot_GPIO6;
            public int Robot_GPIO7;

            public int Robot_ADC_CH1;
            public int Robot_ADC_CH2;
            public int Robot_ADC_CH3;
            public int Robot_ADC_CH4;
            public int Robot_ADC_CH5;
            public int Robot_ADC_CH6;
            public int Robot_ADC_CH7;
            public int Robot_ADC_CH8;

            public int Robot_EXTI1;
            public int Robot_EXTI2;
            public int Robot_EXTI3;
            public int Robot_EXTI4;
            public int Robot_EXTI5;
            public int Robot_EXTI6;
            public int Robot_EXTI7;
            public int Robot_EXTI8;

            public int Robot_PES_Byte1;
            public int Robot_PES_Byte2;
            public int Robot_PES_Byte3;
            public int Robot_PES_Byte4;
            public int Robot_PES_Byte5;
            public int Robot_PES_Byte6;
            public int Robot_PES_Byte7;
            public int Robot_PES_Byte8;

            public int Robot_Encoder1;
            public int Robot_Encoder2;

            public int Compass_X_Reg;
            public int Compass_Y_Reg;
            public int Compass_Z_Reg;

            public int Compass_X_Offset;
            public int Compass_Y_Offset;
            public int Compass_Z_Offset;

            public int Compass_X_Angle;
            public int Compass_Y_Angle;
            public int Compass_Z_Angle;
            public int Compass_KZ_Angle;

            public int Scale1;
            public int Scale2;
            public int Scale3;


            public XyDataSeries<double, double> _xyDataSeries_Robot1;
            public XyDataSeries<double, double> _xyDataSeries_Robot12;
            public XyDataSeries<double, double> _xyDataSeries_Robot2;
            public XyDataSeries<double, double> _xyDataSeries_Robot22;
            public XyDataSeries<double, double> _xyDataSeries_Robot3;
            public XyDataSeries<double, double> _xyDataSeries_Robot4;

            public XyDataSeries<double, double> _xyDataSeries;
            public XyDataSeries<double, double> _xyDataSeries_rb1;

            public XyDataSeries<double, double> _xyDataSeries_rb2;
            public XyDataSeries<double, double> _xyDataSeries_rb3;
            public XyDataSeries<double, double> _xyDataSeries_rb4;
            public XyDataSeries<double, double> _xyDataSeries_rb5;
            public XyDataSeries<double, double> _xyDataSeries_rb6;
            public XyDataSeries<double, double> _xyDataSeries_rb7;
            public XyDataSeries<double, double> _xyDataSeries_rb8;
            public XyDataSeries<double, double> _xyDataSeries_rb9;
            public XyDataSeries<double, double> _xyDataSeries_rb10;
            public XyDataSeries<double, double> _xyDataSeries_rb11;
            public XyDataSeries<double, double> _xyDataSeries_rb12;
            public XyDataSeries<double, double> _xyDataSeries_rb13;

            public XyDataSeries<double, double> _xyDataSeries_IO1;
            public XyDataSeries<double, double> _xyDataSeries_IO2;
            public XyDataSeries<double, double> _xyDataSeries_IO3;
            public XyDataSeries<double, double> _xyDataSeries_IO4;
            public XyDataSeries<double, double> _xyDataSeries_IO5;
            public XyDataSeries<double, double> _xyDataSeries_IO6;
            public XyDataSeries<double, double> _xyDataSeries_IO7;
            public XyDataSeries<double, double> _xyDataSeries_IO8;
            public XyDataSeries<double, double> _xyDataSeries_IO9;

            public XyDataSeries<double, double> _xyDataSeries_R1;
            public XyDataSeries<double, double> _xyDataSeries_R2;
            public XyDataSeries<double, double> _xyDataSeries_R3;
            public XyDataSeries<double, double> _xyDataSeries_R4;
            public XyDataSeries<double, double> _xyDataSeries_A1;
            public XyDataSeries<double, double> _xyDataSeries_A2;
            public XyDataSeries<double, double> _xyDataSeries_A3;
            public XyDataSeries<double, double> _xyDataSeries_A4;
            public XyDataSeries<double, double> _xyDataSeries_A5;

            public DataTable KS;
            public DataRow KS_data1;

            public Robot_Analyzer_def
               (
                 double[] _re1,
                 double[] _im1,

                // vị trí di chuyển
                 double[] _re_R1,
                 double[] _im_R1,

                // duty1
                 double[] _re_R2,
                 double[] _im_R2,

                // duty2
                 double[] _re_R3,
                 double[] _im_R3,

                // duty3
                 double[] _re_R4,
                 double[] _im_R4,

                // duty4
                 double[] _re_R5,
                 double[] _im_R5,

                // ADC1
                 double[] _re_R6,
                 double[] _im_R6,

                // ADC2
                 double[] _re_R7,
                 double[] _im_R7,

                 UInt16 connect,
                 UInt16 Counters,
                 UInt16 Counters2,

                 UInt32 Max_ADC,
                 Int32 Max_Compass,
                 Int32 Min_Compass,
                 UInt32 Max_ABC,

                 UInt32 Max_ADCi,
                 Int32 Max_Compassi,
                 Int32 Min_Compassi,
                 UInt32 Max_ABCi,

                 byte[] Lidar_Frame,
                 byte[] Robo_Frame,
                 byte[] Robo_Frame2,
                 byte[] Compass_Frame,

                 UInt16[] Robot_EEpromi,
                 UInt16[] Robot_EEprom,
                 int EEprom_Mode,
                 int EEprom_Save,
                 int EEprom_Load,
                 int EEprom_Read_Point,
                 int EEprom_Save_file,
                 int EEprom_Load_file,

                int[] Robo_duty,
                 uint[] Robo_dir,
                 int Robo_Encoder,
                 double Robo_Compass,
                 uint Robo_ADC1,
                 uint Robo_ADC2,
                 uint Robo_ADC3,
                 uint Robo_A_Val,
                 uint Robo_B_Val,
                 uint Robo_C_Val,
                 uint Robo_Input_Val,
                 uint Robo_Input_Val2,
                 uint Robo_Input_Val3,
                 uint[] Robo_bit,
                 uint[] Robo_bit2,
                 uint[] Robo_bit3,
                 uint[] Robo_bit4,
                 UInt16 Robo_Sample,
                 UInt16 Robo_Restart,
                 UInt16 Robo_Restart2,
                 byte[] Robo_PES,

                 int Compass_R1,
                 int Compass_R2,
                 int Compass_R3,
                 int Compass_R4,
                 double Compass_A1,
                 double Compass_A2,
                 double Compass_A3,
                 double Compass_A4,
                 double Compass_A5,
                 int Compass_Offset1,
                 int Compass_Offset2,
                 int Compass_Offset3,
                 int Compass_Offset4,

                 int Compass_Raw_Max,
                 int Compass_Raw_Min,

                 double Compass_Result_Max,
                 double Compass_Result_Min,

                 int Compass_Raw_Maxi,
                 int Compass_Raw_Mini,

                 double Compass_Result_Maxi,
                 double Compass_Result_Mini,

                 double Compass_input3_Max,
                 double Compass_input3_Maxi,

                 uint Robot_Point,
                 uint Robot_Running,

                UInt16 Compass_Sample,

                 int Clear_Request1,
                 int Clear_Request2,

                 byte[] Robot_Packet_Frame,
                 int Robot_Packet_Count,
                 int Robot_Packet_View,
                 int Robot_Packet_Step,
                 byte[] Robot_Packet_USB,
                 int Robot_Packet_Save,
                 int Robot_Packet_Save_File,
                 int Robot_Thread_Run,
                 int Robot_Analyse_Start,

                 int Robot_Analyse,
                 int Robot_View_Mode,
                 int Robot_View_Stop,
                 int Robot_View_Stop2,
                 int Robot_View_SYNC,
                 int Robot_View_Clear2,
                 int Robot_View_Cleari2,
                 int Robot_View_Clear,
                 int Robot_View_Cleari,
                 int Robot_View_Delay,

                 int Robot_Counter,
                 int Robot_Clear,
                 int Robot_View1,
                 int Robot_View2,
                 int Robot_View3,
                 int Robot_View4,

                int Robot_GPIO1,
                int Robot_GPIO2,
                int Robot_GPIO3,
                int Robot_GPIO4,
                int Robot_GPIO5,
                int Robot_GPIO6,
                int Robot_GPIO7,

                int Robot_ADC_CH1,
                int Robot_ADC_CH2,
                int Robot_ADC_CH3,
                int Robot_ADC_CH4,
                int Robot_ADC_CH5,
                int Robot_ADC_CH6,
                int Robot_ADC_CH7,
                int Robot_ADC_CH8,

                int Robot_EXTI1,
                int Robot_EXTI2,
                int Robot_EXTI3,
                int Robot_EXTI4,
                int Robot_EXTI5,
                int Robot_EXTI6,
                int Robot_EXTI7,
                int Robot_EXTI8,

                int Robot_PES_Byte1,
                int Robot_PES_Byte2,
                int Robot_PES_Byte3,
                int Robot_PES_Byte4,
                int Robot_PES_Byte5,
                int Robot_PES_Byte6,
                int Robot_PES_Byte7,
                int Robot_PES_Byte8,

                int Robot_Encoder1,
                int Robot_Encoder2,

                int Compass_X_Reg,
                int Compass_Y_Reg,
                int Compass_Z_Reg,

                int Compass_X_Offset,
                int Compass_Y_Offset,
                int Compass_Z_Offset,

                int Compass_X_Angle,
                int Compass_Y_Angle,
                int Compass_Z_Angle,
                int Compass_KZ_Angle,

                int Scale1,
                int Scale2,
                int Scale3,

                XyDataSeries<double, double> _xyDataSeries_Robot1,
                 XyDataSeries<double, double> _xyDataSeries_Robot2,
                 XyDataSeries<double, double> _xyDataSeries_Robot3,
                 XyDataSeries<double, double> _xyDataSeries_Robot4,
                 XyDataSeries<double, double> _xyDataSeries_Robot12,
                 XyDataSeries<double, double> _xyDataSeries_Robot22,

                 XyDataSeries<double, double> _xyDataSeries,
                 XyDataSeries<double, double> _xyDataSeries_rb1,

                 XyDataSeries<double, double> _xyDataSeries_rb2,
                 XyDataSeries<double, double> _xyDataSeries_rb3,
                 XyDataSeries<double, double> _xyDataSeries_rb4,
                 XyDataSeries<double, double> _xyDataSeries_rb5,
                 XyDataSeries<double, double> _xyDataSeries_rb6,
                 XyDataSeries<double, double> _xyDataSeries_rb7,
                 XyDataSeries<double, double> _xyDataSeries_rb8,
                 XyDataSeries<double, double> _xyDataSeries_rb9,
                 XyDataSeries<double, double> _xyDataSeries_rb10,
                 XyDataSeries<double, double> _xyDataSeries_rb11,
                 XyDataSeries<double, double> _xyDataSeries_rb12,
                 XyDataSeries<double, double> _xyDataSeries_rb13,

                 XyDataSeries<double, double> _xyDataSeries_IO1,
                 XyDataSeries<double, double> _xyDataSeries_IO2,
                 XyDataSeries<double, double> _xyDataSeries_IO3,
                 XyDataSeries<double, double> _xyDataSeries_IO4,
                 XyDataSeries<double, double> _xyDataSeries_IO5,
                 XyDataSeries<double, double> _xyDataSeries_IO6,
                 XyDataSeries<double, double> _xyDataSeries_IO7,
                 XyDataSeries<double, double> _xyDataSeries_IO8,
                 XyDataSeries<double, double> _xyDataSeries_IO9,

                 XyDataSeries<double, double> _xyDataSeries_R1,
                 XyDataSeries<double, double> _xyDataSeries_R2,
                 XyDataSeries<double, double> _xyDataSeries_R3,
                 XyDataSeries<double, double> _xyDataSeries_R4,
                 XyDataSeries<double, double> _xyDataSeries_A1,
                 XyDataSeries<double, double> _xyDataSeries_A2,
                 XyDataSeries<double, double> _xyDataSeries_A3,
                 XyDataSeries<double, double> _xyDataSeries_A4,
                 XyDataSeries<double, double> _xyDataSeries_A5,

                 DataTable KS,
                 DataRow KS_data1
            )
            {
                this._re1 = _re1;
                 this._im1= _im1;

                // vị trí di chuyển
                 this._re_R1= _re_R1;
                 this._im_R1= _im_R1;

                // duty1
                 this._re_R2= _re_R2;
                 this._im_R2= _im_R2;

                // duty2
                 this._re_R3= _re_R3;
                 this._im_R3= _im_R3;

                // duty3
                 this._re_R4= _re_R4;
                 this._im_R4= _im_R4;

                // duty4
                 this._re_R5= _re_R5;
                 this._im_R5= _im_R5;

                // ADC1
                 this._re_R6= _re_R6;
                 this._im_R6= _im_R6;

                // ADC2
                 this._re_R7= _re_R7;
                 this._im_R7= _im_R7;

                 this.connect = connect;
                 this.Counters = Counters;
                 this.Counters2 = Counters2;

                 this.Max_ADC = Max_ADC;
                 this.Max_Compass = Max_Compass;
                 this.Min_Compass = Min_Compass;
                 this.Max_ABC = Max_ABC;

                 this.Max_ADCi = Max_ADCi;
                 this.Max_Compassi = Max_Compassi;
                 this.Min_Compassi = Min_Compassi;
                 this.Max_ABCi = Max_ABCi;

                 this.Lidar_Frame= Lidar_Frame;
                 this.Robo_Frame= Robo_Frame;
                 this.Robo_Frame2= Robo_Frame2;
                 this.Compass_Frame= Compass_Frame;

                this.Robot_EEpromi = Robot_EEpromi;
                this.Robot_EEprom = Robot_EEprom;
                this.EEprom_Mode = EEprom_Mode;
                 this.EEprom_Save = EEprom_Save;
                 this.EEprom_Load = EEprom_Load;
                 this.EEprom_Read_Point = EEprom_Read_Point;
                 this.EEprom_Save_file = EEprom_Save_file;
                 this.EEprom_Load_file = EEprom_Load_file;

                 this.Robo_duty= Robo_duty;
                 this.Robo_dir= Robo_dir;
                 this.Robo_Encoder= Robo_Encoder;
                 this.Robo_Compass= Robo_Compass;
                 this.Robo_ADC1= Robo_ADC1;
                 this.Robo_ADC2= Robo_ADC2;
                 this.Robo_ADC3= Robo_ADC3;
                 this.Robo_A_Val= Robo_A_Val;
                 this.Robo_B_Val= Robo_B_Val;
                 this.Robo_C_Val= Robo_C_Val;
                 this.Robo_Input_Val= Robo_Input_Val;
                 this.Robo_Input_Val2= Robo_Input_Val2;
                 this.Robo_Input_Val3= Robo_Input_Val3;
                 this.Robo_bit= Robo_bit;
                 this.Robo_bit2= Robo_bit2;
                 this.Robo_bit3= Robo_bit3;
                 this.Robo_bit4= Robo_bit4;
                 this.Robo_Sample= Robo_Sample;
                 this.Robo_Restart= Robo_Restart;
                 this.Robo_Restart2= Robo_Restart2;
                 this.Robo_PES= Robo_PES;

                 this.Compass_R1= Compass_R1;
                 this.Compass_R2= Compass_R2;
                 this.Compass_R3= Compass_R3;
                 this.Compass_R4= Compass_R4;
                 this.Compass_A1= Compass_A1;
                 this.Compass_A2= Compass_A2;
                 this.Compass_A3= Compass_A3;
                 this.Compass_A4= Compass_A4;
                 this.Compass_A5= Compass_A5;
                 this.Compass_Offset1= Compass_Offset1;
                 this.Compass_Offset2= Compass_Offset2;
                 this.Compass_Offset3= Compass_Offset3;
                 this.Compass_Offset4= Compass_Offset4;

                 this.Compass_Raw_Max= Compass_Raw_Max;
                 this.Compass_Raw_Min= Compass_Raw_Min;

                 this.Compass_Result_Max= Compass_Result_Max;
                 this.Compass_Result_Min= Compass_Result_Min;

                 this.Compass_Raw_Maxi= Compass_Raw_Maxi;
                 this.Compass_Raw_Mini= Compass_Raw_Mini;

                 this.Compass_Result_Maxi= Compass_Result_Maxi;
                 this.Compass_Result_Mini= Compass_Result_Mini;

                 this.Compass_input3_Max= Compass_input3_Max;
                 this.Compass_input3_Maxi= Compass_input3_Maxi;

                 this.Robot_Point= Robot_Point;
                 this.Robot_Running = Robot_Running;

                 this.Compass_Sample= Compass_Sample;

                 this.Clear_Request1= Clear_Request1;
                 this.Clear_Request2= Clear_Request2;

                 this.Robot_Packet_Frame = Robot_Packet_Frame;
                 this.Robot_Packet_Count = Robot_Packet_Count;
                 this.Robot_Packet_View = Robot_Packet_View;
                 this.Robot_Packet_Step = Robot_Packet_Step;
                 this.Robot_Packet_USB = Robot_Packet_USB;
                 this.Robot_Packet_Save = Robot_Packet_Save;
                 this.Robot_Packet_Save_File = Robot_Packet_Save_File;
                 this.Robot_Thread_Run = Robot_Thread_Run;
                 this.Robot_Analyse_Start = Robot_Analyse_Start;

                 this.Robot_Analyse= Robot_Analyse;
                 this.Robot_View_Mode= Robot_View_Mode;
                 this.Robot_View_Stop= Robot_View_Stop;
                 this.Robot_View_Stop2= Robot_View_Stop2;
                 this.Robot_View_SYNC= Robot_View_SYNC;
                 this.Robot_View_Clear2= Robot_View_Clear2;
                 this.Robot_View_Cleari2= Robot_View_Cleari2;
                 this.Robot_View_Clear= Robot_View_Clear;
                 this.Robot_View_Cleari= Robot_View_Cleari;
                 this.Robot_View_Delay= Robot_View_Delay;

                this.Robot_Counter = Robot_Counter;
                this.Robot_Clear = Robot_Clear;
                this.Robot_View1 = Robot_View1;
                this.Robot_View2 = Robot_View2;
                this.Robot_View3 = Robot_View3;
                this.Robot_View4 = Robot_View4;

                this.Robot_GPIO1 = Robot_GPIO1;
                this.Robot_GPIO2 = Robot_GPIO2;
                this.Robot_GPIO3 = Robot_GPIO3;
                this.Robot_GPIO4 = Robot_GPIO4;
                this.Robot_GPIO5 = Robot_GPIO5;
                this.Robot_GPIO6 = Robot_GPIO6;
                this.Robot_GPIO7 = Robot_GPIO7;

                this.Robot_ADC_CH1 = Robot_ADC_CH1;
                this.Robot_ADC_CH2 = Robot_ADC_CH2;
                this.Robot_ADC_CH3 = Robot_ADC_CH3;
                this.Robot_ADC_CH4 = Robot_ADC_CH4;
                this.Robot_ADC_CH5 = Robot_ADC_CH5;
                this.Robot_ADC_CH6 = Robot_ADC_CH6;
                this.Robot_ADC_CH7 = Robot_ADC_CH7;
                this.Robot_ADC_CH8 = Robot_ADC_CH8;

                this.Robot_EXTI1 = Robot_EXTI1;
                this.Robot_EXTI2 = Robot_EXTI2;
                this.Robot_EXTI3 = Robot_EXTI3;
                this.Robot_EXTI4 = Robot_EXTI4;
                this.Robot_EXTI5 = Robot_EXTI5;
                this.Robot_EXTI6 = Robot_EXTI6;
                this.Robot_EXTI7 = Robot_EXTI7;
                this.Robot_EXTI8 = Robot_EXTI8;

                this.Robot_PES_Byte1 = Robot_PES_Byte1;
                this.Robot_PES_Byte2 = Robot_PES_Byte2;
                this.Robot_PES_Byte3 = Robot_PES_Byte3;
                this.Robot_PES_Byte4 = Robot_PES_Byte4;
                this.Robot_PES_Byte5 = Robot_PES_Byte5;
                this.Robot_PES_Byte6 = Robot_PES_Byte6;
                this.Robot_PES_Byte7 = Robot_PES_Byte7;
                this.Robot_PES_Byte8 = Robot_PES_Byte8;

                this.Robot_Encoder1 = Robot_Encoder1;
                this.Robot_Encoder2 = Robot_Encoder2;

                this.Compass_X_Reg = Compass_X_Reg;
                this.Compass_Y_Reg = Compass_Y_Reg;
                this.Compass_Z_Reg = Compass_Z_Reg;

                this.Compass_X_Offset = Compass_X_Offset;
                this.Compass_Y_Offset = Compass_Y_Offset;
                this.Compass_Z_Offset = Compass_Z_Offset;

                this.Compass_X_Angle = Compass_X_Angle;
                this.Compass_Y_Angle = Compass_Y_Angle;
                this.Compass_Z_Angle = Compass_Z_Angle;
                this.Compass_KZ_Angle = Compass_KZ_Angle;

                this.Scale1 = Scale1;
                this.Scale2 = Scale2;
                this.Scale3 = Scale3;

                this._xyDataSeries_Robot1 = _xyDataSeries_Robot1;
                this._xyDataSeries_Robot2 = _xyDataSeries_Robot2;
                this._xyDataSeries_Robot3 = _xyDataSeries_Robot3;
                this._xyDataSeries_Robot4 = _xyDataSeries_Robot4;
                this._xyDataSeries_Robot12 = _xyDataSeries_Robot12;
                this._xyDataSeries_Robot22 = _xyDataSeries_Robot22;

                this._xyDataSeries= _xyDataSeries;
                 this._xyDataSeries_rb1= _xyDataSeries_rb1;

                 this._xyDataSeries_rb2= _xyDataSeries_rb2;
                 this._xyDataSeries_rb3= _xyDataSeries_rb3;
                 this._xyDataSeries_rb4= _xyDataSeries_rb4;
                 this._xyDataSeries_rb5= _xyDataSeries_rb5;
                 this._xyDataSeries_rb6= _xyDataSeries_rb6;
                 this._xyDataSeries_rb7= _xyDataSeries_rb7;
                 this._xyDataSeries_rb8= _xyDataSeries_rb8;
                 this._xyDataSeries_rb9= _xyDataSeries_rb9;
                 this._xyDataSeries_rb10= _xyDataSeries_rb10;
                 this._xyDataSeries_rb11= _xyDataSeries_rb11;
                 this._xyDataSeries_rb12= _xyDataSeries_rb12;
                 this._xyDataSeries_rb13= _xyDataSeries_rb13;

                 this._xyDataSeries_IO1= _xyDataSeries_IO1;
                 this._xyDataSeries_IO2= _xyDataSeries_IO2;
                 this._xyDataSeries_IO3= _xyDataSeries_IO3;
                 this._xyDataSeries_IO4= _xyDataSeries_IO4;
                 this._xyDataSeries_IO5= _xyDataSeries_IO5;
                 this._xyDataSeries_IO6= _xyDataSeries_IO6;
                 this._xyDataSeries_IO7= _xyDataSeries_IO7;
                 this._xyDataSeries_IO8= _xyDataSeries_IO8;
                 this._xyDataSeries_IO9= _xyDataSeries_IO9;

                 this._xyDataSeries_R1= _xyDataSeries_R1;
                 this._xyDataSeries_R2= _xyDataSeries_R2;
                 this._xyDataSeries_R3= _xyDataSeries_R3;
                 this._xyDataSeries_R4= _xyDataSeries_R4;
                 this._xyDataSeries_A1= _xyDataSeries_A1;
                 this._xyDataSeries_A2= _xyDataSeries_A2;
                 this._xyDataSeries_A3= _xyDataSeries_A3;
                 this._xyDataSeries_A4 = _xyDataSeries_A4;
                 this._xyDataSeries_A5 = _xyDataSeries_A5;

                 this.KS = KS;
                 this.KS_data1 = KS_data1;
            }
        };

        public struct PID_struct_def
        {
            public byte[] USD_HID_Frame;
            public byte[] USD_HID_Frameo;
            public byte[] USD_HID_Config;

            public int HID_VendorId;
            public int HID_productId;

            public int HID_Start;
            public int Reconect_Start;
            public int Disconect_Time2;
            public int HID_Done;
            public int Disconect_Time;

            public int Com_counter;
            public int Connect_ok;
            public int Send_Request;
            public int Send_Counter;
            public int Send_Start;
            public int Send_Stop;
            public int Send_Status;
            public int PID_Control_Mode;
            public int PID_Control_lOAD;
            public int PID_ENC_Counter;
            public int _Mesure_Dir;
            public int PID_ENC_OverFlow;
            public byte[] Send_Data;


            public PID_struct_def
                (
                byte[] USD_HID_Frame,
                byte[] USD_HID_Frameo,
                byte[] USD_HID_Config,

                int HID_VendorId,
                int HID_productId,

                int HID_Start,
                int Reconect_Start,
                int Disconect_Time2,
                int HID_Done,
                int Disconect_Time,
          

                int Com_counter,
                int Connect_ok,
                int Send_Request,
                int Send_Counter,
                int Send_Start,
                int Send_Stop,
                int Send_Status,
                int PID_Control_Mode,
                int PID_Control_lOAD,
                int PID_ENC_Counter,
                int PID_ENC_OverFlow,
                int _Mesure_Dir,
                byte[] Send_Data
                )
            {
                this.USD_HID_Frame = USD_HID_Frame;
                this.USD_HID_Frameo = USD_HID_Frameo;
                this.USD_HID_Config = USD_HID_Config;

                this.HID_VendorId = HID_VendorId;
                this.HID_productId = HID_productId;

                this.HID_Start = HID_Start;
                this.Reconect_Start = Reconect_Start;
                this.Disconect_Time2 = Disconect_Time2;
                this.HID_Done = HID_Done;
                this.Disconect_Time = Disconect_Time;

                this.Com_counter = Com_counter;
                this.Connect_ok = Connect_ok;
                this.Send_Request = Send_Request;
                this.Send_Counter = Send_Counter;
                this.Send_Start = Send_Start;
                this.Send_Stop = Send_Stop;
                this.Send_Status = Send_Status;
                this.PID_Control_Mode = PID_Control_Mode;
                this.PID_Control_lOAD = PID_Control_lOAD;
                this.PID_ENC_Counter = PID_ENC_Counter;
                this.PID_ENC_OverFlow = PID_ENC_OverFlow;
                this._Mesure_Dir = _Mesure_Dir;
                this.Send_Data = Send_Data;

               
            }
        };


        #endregion

        public MainWindow()
        {
            InitializeComponent();
            Innit_Struct_data();
            Innit_Chart();
            _PID_Innit();
            Innit_Thread();
            Innit_Grid();
            Robot_Analyzers.Robot_Running = 1;

            Setup_eeprom.Visibility = Visibility.Hidden;
        }

        #region Innit Data
        
        public void Innit_Chart()
        {
            xAxisAlignment.ItemsSource = axisAlignment;
            xAxisAlignment.SelectedItem = Enum.GetName(typeof(AxisAlignment), AxisAlignment.Bottom);

            yAxisAlignment.ItemsSource = axisAlignment;
            yAxisAlignment.SelectedItem = Enum.GetName(typeof(AxisAlignment), AxisAlignment.Left);


            //var dataSeries = new XyDataSeries<double, double>();

            lineRenderSeries.DataSeries = Robot_Analyzers._xyDataSeries;



            var data = DataManager.Instance.GetSquirlyWave();

            for (int i = 0; i < 360; i++)
            {
                Robot_Analyzers._re1[i] = i * 20 / 360.0;
                Robot_Analyzers._im1[i] = i;
            }

            //Sensor_Data_move.Dispatcher.Invoke(() => Sensor_Data_move.DataSeries = _xyDataSeries_rb1);

            for (int i = 0; i < 20000; i++)
            {
                Robot_Analyzers._re_R1[i] = 0;
                Robot_Analyzers._im_R1[i] = i;
            }

            Robot_Analyzers._xyDataSeries_rb1.Clear();
            Robot_Analyzers._xyDataSeries_rb1.Append(Robot_Analyzers._re_R1, Robot_Analyzers._im_R1);

            // Append data to series. SciChart automatically redraws
            //dataSeries.Append(data.XData, data.YData);
            Robot_Analyzers._xyDataSeries.Clear();
            Robot_Analyzers._xyDataSeries.Append(Robot_Analyzers._im1, Robot_Analyzers._re1);

            sciChart.ZoomExtents();
        }
        
        public void Innit_Thread()
        {
            //Thread_ethenet = new Thread(() => Lidar_Signal());
            //Thread_ethenet.Start();

            Thread_ethenet2 = new Thread(() => Robo_Signal());
            Thread_ethenet2.Start();

            //Thread_ethenet3 = new Thread(() => Compass_Signal());
            //Thread_ethenet3.Start();

            //Thread_ethenet4 = new Thread(() => Robo_Signal2());
            //Thread_ethenet4.Start();

            //Thread_ethenet5 = new Thread(() => Robo_Analyse());
            //Thread_ethenet5.Start();

            Thread_USB = new Thread(() => USB_Bulk_Handle2());
            Thread_USB.Start();

            Playback_time.Tick += Playback_time_Tick;
            Playback_time.Interval = new TimeSpan(0, 0, 0, 0, 100);   // 1s duoc thư hien 1 lan
            Playback_time.Stop();

            Check_USB.Tick += Check_USB_Tick;
            Check_USB.Interval = new TimeSpan(0, 0, 0, 0, 100);  // Mili giây
            Check_USB.Start();

            Check_Counter.Tick += Check_Counter_Tick;
            Check_Counter.Interval = new TimeSpan(0, 0, 0, 0, 200);   // Mili giây
            Check_Counter.Start();

        }

        public void Innit_Grid()
        {
            ROBOT_VIEW.Visibility = Visibility.Visible;
            Lidar_Viewx.Visibility = Visibility.Hidden;
            Lidar_View2.Visibility = Visibility.Hidden;
        }

        public void Innit_Struct_data()
        {
            Robot_Analyzers._re1 = new double[360];
            Robot_Analyzers._im1 = new double[360];

            // vị trí di chuyển
            Robot_Analyzers._re_R1 = new double[20000];
            Robot_Analyzers._im_R1 = new double[20000];

            // duty1
            Robot_Analyzers._re_R2 = new double[20000];
            Robot_Analyzers._im_R2 = new double[20000];

            // duty2
            Robot_Analyzers._re_R3 = new double[20000];
            Robot_Analyzers._im_R3 = new double[20000];

            // duty3
            Robot_Analyzers._re_R4 = new double[20000];
            Robot_Analyzers._im_R4 = new double[20000];

            // duty4
            Robot_Analyzers._re_R5 = new double[20000];
            Robot_Analyzers._im_R5 = new double[20000];

            // ADC1
            Robot_Analyzers._re_R6 = new double[20000];
            Robot_Analyzers._im_R6 = new double[20000];

            // ADC2
            Robot_Analyzers._re_R7 = new double[20000];
            Robot_Analyzers._im_R7 = new double[20000];

            Robot_Analyzers.Lidar_Frame = new byte[800];
            Robot_Analyzers.Robo_Frame = new byte[80];
            Robot_Analyzers.Robo_Frame2 = new byte[80];
            Robot_Analyzers.Compass_Frame = new byte[80];

            Robot_Analyzers.Robo_duty = new int[4];
            Robot_Analyzers.Robo_dir = new uint[4];
            Robot_Analyzers.Robo_Encoder = 0;
            Robot_Analyzers.Robo_Compass = 0;
            Robot_Analyzers.Robo_ADC1 = 0;
            Robot_Analyzers.Robo_ADC2 = 0;
            Robot_Analyzers.Robo_ADC3 = 0;
            Robot_Analyzers.Robo_A_Val = 0;
            Robot_Analyzers.Robo_B_Val = 0;
            Robot_Analyzers.Robo_C_Val = 0;
            Robot_Analyzers.Robo_Input_Val = 0;
            Robot_Analyzers.Robo_Input_Val2 = 0;
            Robot_Analyzers.Robo_Input_Val3 = 0;
            Robot_Analyzers.Robo_bit = new uint[8];
            Robot_Analyzers.Robo_bit2 = new uint[8];
            Robot_Analyzers.Robo_bit3 = new uint[8];
            Robot_Analyzers.Robo_bit4 = new uint[8];
            Robot_Analyzers.Robo_Sample = 0;
            Robot_Analyzers.Robo_Restart = 0;
            Robot_Analyzers.Robo_Restart2 = 0;
            Robot_Analyzers.Robo_PES = new byte[6];

            Robot_Analyzers.Robot_EEprom = new UInt16[20];
            Robot_Analyzers.Robot_EEpromi = new UInt16[20];
            Robot_Analyzers.EEprom_Mode = 0;
            Robot_Analyzers.EEprom_Save = 0;
            Robot_Analyzers.EEprom_Load = 0;
            Robot_Analyzers.EEprom_Save_file = 0;
            Robot_Analyzers.EEprom_Load_file = 0;

            Robot_Analyzers.Compass_R1 = 0;
            Robot_Analyzers.Compass_R2 = 0;
            Robot_Analyzers.Compass_R3 = 0;
            Robot_Analyzers.Compass_R4 = 0;
            Robot_Analyzers.Compass_A1 = 0;
            Robot_Analyzers.Compass_A2 = 0;
            Robot_Analyzers.Compass_A3 = 0;
            Robot_Analyzers.Compass_A4 = 0;
            Robot_Analyzers.Compass_A5 = 0;
            Robot_Analyzers.Compass_Offset1 = 0;
            Robot_Analyzers.Compass_Offset2 = 0;
            Robot_Analyzers.Compass_Offset3 = 0;
            Robot_Analyzers.Compass_Offset4 = 0;

            Robot_Analyzers.Compass_Raw_Max = 10;
            Robot_Analyzers.Compass_Raw_Min = -10;

            Robot_Analyzers.Compass_Result_Max = 10;
            Robot_Analyzers.Compass_Result_Min = -10;

            Robot_Analyzers.Compass_Raw_Maxi = 10;
            Robot_Analyzers.Compass_Raw_Mini = -10;

            Robot_Analyzers.Compass_Result_Maxi = 10;
            Robot_Analyzers.Compass_Result_Mini = -10;

            Robot_Analyzers.Compass_input3_Max = 1.2;
            Robot_Analyzers.Compass_input3_Maxi = 1.2;

            Robot_Analyzers.Robot_Point = 0;
            Robot_Analyzers.Robot_Running = 0;

            Robot_Analyzers.Compass_Sample = 0;

            Robot_Analyzers.Clear_Request1 = 0;
            Robot_Analyzers.Clear_Request2 = 0;

            Robot_Analyzers.Robot_Packet_Frame = new byte[32 * 20000];
            Robot_Analyzers.Robot_Packet_Count = 0;
            Robot_Analyzers.Robot_Packet_View = 0;
            Robot_Analyzers.Robot_Packet_Step = 0;
            Robot_Analyzers.Robot_Packet_USB = new byte[32];
            Robot_Analyzers.Robot_Packet_Save = 0;
            Robot_Analyzers.Robot_Packet_Save_File = 0;
            Robot_Analyzers.Robot_Thread_Run = 0;
            Robot_Analyzers.Robot_Analyse_Start = 0;

            Robot_Analyzers.Robot_Analyse = 0;
            Robot_Analyzers.Robot_View_Mode = 0;
            Robot_Analyzers.Robot_View_Stop = 0;
            Robot_Analyzers.Robot_View_Stop2 = 0;
            Robot_Analyzers.Robot_View_SYNC = 0;
            Robot_Analyzers.Robot_View_Clear2 = 0;
            Robot_Analyzers.Robot_View_Cleari2 = 40;
            Robot_Analyzers.Robot_View_Clear = 0;
            Robot_Analyzers.Robot_View_Cleari = 40;
            Robot_Analyzers.Robot_View_Delay = 10;

            Robot_Analyzers._xyDataSeries_Robot1 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_Robot2 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_Robot3 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_Robot4 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_Robot12 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_Robot22 = new XyDataSeries<double>();

            Robot_Analyzers._xyDataSeries = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb1 = new XyDataSeries<double>();

            Robot_Analyzers._xyDataSeries_rb2 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb3 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb4 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb5 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb6 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb7 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb8 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb9 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb10 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb11 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb12 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb13 = new XyDataSeries<double>();

            Robot_Analyzers._xyDataSeries_IO1 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_IO2 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_IO3 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_IO4 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_IO5 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_IO6 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_IO7 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_IO8 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_IO9 = new XyDataSeries<double>();

            Robot_Analyzers._xyDataSeries_R1 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_R2 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_R3 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_R4 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_A1 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_A2 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_A3 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_A4 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_A5 = new XyDataSeries<double>();

            Robot_Analyzers.KS = new DataTable();
        //Robot_Analyzers.KS_data1;
    }

        #endregion


        #region USB

        private void _PID_Innit()
        {
            PID_struct.USD_HID_Frame = new byte[64];
            PID_struct.USD_HID_Frameo = new byte[64];
            PID_struct.USD_HID_Config = new byte[64];

            PID_struct.Com_counter = 0;
            PID_struct.Connect_ok = 0;
            PID_struct.Send_Request = 0;
            PID_struct.Send_Counter = 0;
            PID_struct.Send_Start = 0;
            PID_struct.Send_Stop = 0;
            PID_struct.Send_Status = 0;
            PID_struct.PID_Control_Mode = 0;
            PID_struct.PID_Control_lOAD = 0;
            PID_struct.PID_ENC_Counter = 0;
            PID_struct.PID_ENC_OverFlow = 0;
            PID_struct.Send_Data = new byte[3];

            PID_struct.HID_VendorId = 0x0074;
            PID_struct.HID_productId = 0x0074;

            comboBox_u1.Items.Add("Hiển thị ADC1");
            comboBox_u1.Items.Add("Hiển thị ADC2");
            comboBox_u1.Items.Add("Hiển thị ADC3");
            comboBox_u1.Items.Add("Hiển thị ADC4");
            comboBox_u1.Items.Add("Hiển thị ADC5");
            comboBox_u1.Items.Add("Hiển thị ADC6");
            comboBox_u1.Items.Add("Hiển thị ADC7");
            comboBox_u1.Items.Add("Hiển thị ADC8");
            comboBox_u1.Items.Add("Compass X reg");
            comboBox_u1.Items.Add("Compass Y reg");
            comboBox_u1.Items.Add("Compass Z reg");
            comboBox_u1.Items.Add("Compass X Offset");
            comboBox_u1.Items.Add("Compass Y Offset");
            comboBox_u1.Items.Add("Compass Z Offset");
            comboBox_u1.Items.Add("Compass X Angle");
            comboBox_u1.Items.Add("Compass Y Angle");
            comboBox_u1.Items.Add("Compass Z Angle");
            comboBox_u1.Items.Add("Compass Kz Angle");

            comboBox_u11.Items.Add("Hiển thị ADC1");
            comboBox_u11.Items.Add("Hiển thị ADC2");
            comboBox_u11.Items.Add("Hiển thị ADC3");
            comboBox_u11.Items.Add("Hiển thị ADC4");
            comboBox_u11.Items.Add("Hiển thị ADC5");
            comboBox_u11.Items.Add("Hiển thị ADC6");
            comboBox_u11.Items.Add("Hiển thị ADC7");
            comboBox_u11.Items.Add("Hiển thị ADC8");
            comboBox_u11.Items.Add("Compass X reg");
            comboBox_u11.Items.Add("Compass Y reg");
            comboBox_u11.Items.Add("Compass Z reg");
            comboBox_u11.Items.Add("Compass X Offset");
            comboBox_u11.Items.Add("Compass Y Offset");
            comboBox_u11.Items.Add("Compass Z Offset");
            comboBox_u11.Items.Add("Compass X Angle");
            comboBox_u11.Items.Add("Compass Y Angle");
            comboBox_u11.Items.Add("Compass Z Angle");
            comboBox_u11.Items.Add("Compass Kz Angle");

            comboBox_u2.Items.Add("Hiển thị ADC1");
            comboBox_u2.Items.Add("Hiển thị ADC2");
            comboBox_u2.Items.Add("Hiển thị ADC3");
            comboBox_u2.Items.Add("Hiển thị ADC4");
            comboBox_u2.Items.Add("Hiển thị ADC5");
            comboBox_u2.Items.Add("Hiển thị ADC6");
            comboBox_u2.Items.Add("Hiển thị ADC7");
            comboBox_u2.Items.Add("Hiển thị ADC8");
            comboBox_u2.Items.Add("Compass X reg");
            comboBox_u2.Items.Add("Compass Y reg");
            comboBox_u2.Items.Add("Compass Z reg");
            comboBox_u2.Items.Add("Compass X Offset");
            comboBox_u2.Items.Add("Compass Y Offset");
            comboBox_u2.Items.Add("Compass Z Offset");
            comboBox_u2.Items.Add("Compass X Angle");
            comboBox_u2.Items.Add("Compass Y Angle");
            comboBox_u2.Items.Add("Compass Z Angle");
            comboBox_u2.Items.Add("Compass Kz Angle");

            comboBox_u22.Items.Add("Hiển thị ADC1");
            comboBox_u22.Items.Add("Hiển thị ADC2");
            comboBox_u22.Items.Add("Hiển thị ADC3");
            comboBox_u22.Items.Add("Hiển thị ADC4");
            comboBox_u22.Items.Add("Hiển thị ADC5");
            comboBox_u22.Items.Add("Hiển thị ADC6");
            comboBox_u22.Items.Add("Hiển thị ADC7");
            comboBox_u22.Items.Add("Hiển thị ADC8");
            comboBox_u22.Items.Add("Compass X reg");
            comboBox_u22.Items.Add("Compass Y reg");
            comboBox_u22.Items.Add("Compass Z reg");
            comboBox_u22.Items.Add("Compass X Offset");
            comboBox_u22.Items.Add("Compass Y Offset");
            comboBox_u22.Items.Add("Compass Z Offset");
            comboBox_u22.Items.Add("Compass X Angle");
            comboBox_u22.Items.Add("Compass Y Angle");
            comboBox_u22.Items.Add("Compass Z Angle");
            comboBox_u22.Items.Add("Compass Kz Angle");

            comboBox_u1.SelectedIndex = 0;
            comboBox_u11.SelectedIndex = 1;

            comboBox_u2.SelectedIndex = 2;
            comboBox_u22.SelectedIndex = 3;

            comboBox_Scale1.Items.Add("+/-5");
            comboBox_Scale1.Items.Add("+/-10");
            comboBox_Scale1.Items.Add("+/-50");
            comboBox_Scale1.Items.Add("+/-100");
            comboBox_Scale1.Items.Add("+/-250");
            comboBox_Scale1.Items.Add("+/-500");
            comboBox_Scale1.Items.Add("+/-1000");
            comboBox_Scale1.Items.Add("+/-2000");
            comboBox_Scale1.Items.Add("+/-4000");
            comboBox_Scale1.Items.Add("+/-8000");
            comboBox_Scale1.Items.Add("+/-16000");
            comboBox_Scale1.Items.Add("+/-32000");

            comboBox_Scale2.Items.Add("+/-5");
            comboBox_Scale2.Items.Add("+/-10");
            comboBox_Scale2.Items.Add("+/-50");
            comboBox_Scale2.Items.Add("+/-100");
            comboBox_Scale2.Items.Add("+/-250");
            comboBox_Scale2.Items.Add("+/-500");
            comboBox_Scale2.Items.Add("+/-1000");
            comboBox_Scale2.Items.Add("+/-2000");
            comboBox_Scale2.Items.Add("+/-4000");
            comboBox_Scale2.Items.Add("+/-8000");
            comboBox_Scale2.Items.Add("+/-16000");
            comboBox_Scale2.Items.Add("+/-32000");

            comboBox_Scale3.Items.Add("+/-5");
            comboBox_Scale3.Items.Add("+/-10");
            comboBox_Scale3.Items.Add("+/-50");
            comboBox_Scale3.Items.Add("+/-100");
            comboBox_Scale3.Items.Add("+/-250");
            comboBox_Scale3.Items.Add("+/-500");
            comboBox_Scale3.Items.Add("+/-1000");
            comboBox_Scale3.Items.Add("+/-2000");
            comboBox_Scale3.Items.Add("+/-4000");
            comboBox_Scale3.Items.Add("+/-8000");
            comboBox_Scale3.Items.Add("+/-16000");
            comboBox_Scale3.Items.Add("+/-32000");

            comboBox_Scale1.SelectedIndex = 4;
            comboBox_Scale2.SelectedIndex = 4;
            comboBox_Scale3.SelectedIndex = 2;
        }

        private void Check_Counter_Tick(object sender, EventArgs e)
        {
            PID_struct.Disconect_Time++;
            if (PID_struct.Disconect_Time > 20)
            {
                PID_struct.HID_Start = 0;
                PID_struct.Disconect_Time = 0;
                Thread_USB.Abort();
                PID_struct.Reconect_Start = 1;
                PID_struct.Disconect_Time2 = 0;
                label_usb.Content = "Mất kết nối USB!";
            }

            if (PID_struct.Reconect_Start == 1)
            {
                PID_struct.HID_Start = 0;

                label_usb.Content = "kết nối lại USB!";
                PID_struct.Disconect_Time2++;
                if (PID_struct.Disconect_Time2 > 8)
                {
                    PID_struct.Disconect_Time2 = 0;
                    PID_struct.Reconect_Start = 0;
                    Thread_USB = new Thread(() => USB_Bulk_Handle2());
                    Thread_USB.Start();
                }
            }
        }

        private void Check_USB_Tick(object sender, EventArgs e)
        {

            if (PID_struct.HID_Start == 1)
            {
                //Robot_Analyzers.Robot_EEprom = new UInt16[20];
                //Robot_Analyzers.EEprom_Save = 0;
                //Robot_Analyzers.EEprom_Load = 0;
                //Robot_Analyzers.EEprom_Save_file = 0;
                //Robot_Analyzers.EEprom_Load_file = 0;
                

                PID_struct.USD_HID_Frame[4] = (byte)(Robot_Analyzers.EEprom_Save);// 
                PID_struct.USD_HID_Frame[5] = (byte)0;// 

                for(int i=0; i < 20; i++)
                {
                    PID_struct.USD_HID_Frame[i * 2 + 6] = (byte)(Robot_Analyzers.Robot_EEprom[i] / 256);// 
                    PID_struct.USD_HID_Frame[i * 2 + 7] = (byte)(Robot_Analyzers.Robot_EEprom[i] % 256);// 
                }    

                PID_struct.USD_HID_Frame[46] = (byte)(0);//
                PID_struct.USD_HID_Frame[47] = (byte)(0);//
                PID_struct.USD_HID_Frame[48] = (byte)(0);//
                PID_struct.USD_HID_Frame[49] = (byte)(0);//
                PID_struct.USD_HID_Frame[50] = (byte)(0);//
                PID_struct.USD_HID_Frame[51] = (byte)(0);//
                PID_struct.USD_HID_Frame[52] = (byte)(0);//
                PID_struct.USD_HID_Frame[53] = (byte)(0);//
                PID_struct.USD_HID_Frame[54] = (byte)(0);//
                PID_struct.USD_HID_Frame[55] = (byte)(0);// 
                PID_struct.USD_HID_Frame[56] = (byte)(0);//
                PID_struct.USD_HID_Frame[57] = (byte)(0);//
                PID_struct.USD_HID_Frame[58] = (byte)(0);//
                PID_struct.USD_HID_Frame[59] = (byte)(0);//

                try
                {
                    if (device.IsConnected == true)
                    {
                        device.Write(PID_struct.USD_HID_Frame, 100);
                        
                        if(Robot_Analyzers.EEprom_Mode == 1)
                        {
                            if (Robot_Analyzers.EEprom_Save == 2) Robot_Analyzers.EEprom_Save = 1;
                        }
                    }
                }
                catch
                {
                }
            }
        }

        public void USB_Bulk_Handle2()
        {

            uint Size1 = 0; //, Try1 = 0;
            int status = 0;
            int Show_Val1 = 0;
            int Show_Val2 = 0;
            int Show_Val3 = 0;
            int Show_Val4 = 0;
            int Show_Val5 = 0;
            int Show_Val6 = 0;

            int Show_Counter = 0;

            const int HID_VendorId = 0x0088;
            const int HID_productId = 0x0088;

            Robot_Analyzers.Robot_Counter = 0;
            Robot_Analyzers.Robot_Clear = 0;

            Sensor_U1.Dispatcher.Invoke(() => Sensor_U1.DataSeries = Robot_Analyzers._xyDataSeries_Robot1);
            Sensor_U2.Dispatcher.Invoke(() => Sensor_U2.DataSeries = Robot_Analyzers._xyDataSeries_Robot2);
            Sensor_U3.Dispatcher.Invoke(() => Sensor_U3.DataSeries = Robot_Analyzers._xyDataSeries_Robot3);
            Sensor_U12.Dispatcher.Invoke(() => Sensor_U12.DataSeries = Robot_Analyzers._xyDataSeries_Robot12);
            Sensor_U22.Dispatcher.Invoke(() => Sensor_U22.DataSeries = Robot_Analyzers._xyDataSeries_Robot22);
            Sensor_U4.Dispatcher.Invoke(() => Sensor_U4.DataSeries = Robot_Analyzers._xyDataSeries_Robot4);

            while (true)
            {
                try
                {
                    device = HidDevices.Enumerate(HID_VendorId, HID_productId).FirstOrDefault();

                    if (device == null)
                    {
                        Thread.Sleep(200);
                        Size1++;
                        if (Size1 < 20)
                        {
                        }
                        else
                        {
                            Size1 = 0;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                catch
                {
                    Thread.Sleep(100);
                }
            }


            PID_struct.USD_HID_Config[0] = 0xAA;
            PID_struct.USD_HID_Config[1] = 0xBB;

            for (int J = 2; J < 64; J++)
            {
                PID_struct.USD_HID_Frame[J] = 0x11;
                PID_struct.USD_HID_Config[J] = 0x00;
            }

            device.ReadReport(ReadReportCallback);

            label_usb.Dispatcher.Invoke(() => label_usb.Content = "Start Usb");

            while (true)
            {
                PID_struct.HID_Start = 1;
                //if (PID_struct.HID_Exit == 1) break;

                if (PID_struct.HID_Done == 1)
                {
                    PID_struct.HID_Done = 0;
                    if ((HID_report.Data[0] == 'N') && (HID_report.Data[1] == 'A'))
                    {
                        PID_struct.Disconect_Time = 0;

                        Robot_Analyzers.Robot_GPIO1 = HID_report.Data[2];
                        Robot_Analyzers.Robot_GPIO2 = HID_report.Data[3];
                        Robot_Analyzers.Robot_GPIO3 = HID_report.Data[4];
                        Robot_Analyzers.Robot_GPIO4 = HID_report.Data[5];
                        Robot_Analyzers.Robot_GPIO5 = HID_report.Data[6];
                        Robot_Analyzers.Robot_GPIO6 = HID_report.Data[7];
                        Robot_Analyzers.Robot_GPIO7 = HID_report.Data[8];

                        Robot_Analyzers.Robot_ADC_CH1 = HID_report.Data[9] * 256 + HID_report.Data[10];
                        Robot_Analyzers.Robot_ADC_CH2 = HID_report.Data[11] * 256 + HID_report.Data[12];
                        Robot_Analyzers.Robot_ADC_CH3 = HID_report.Data[13] * 256 + HID_report.Data[14];
                        Robot_Analyzers.Robot_ADC_CH4 = HID_report.Data[15] * 256 + HID_report.Data[16];
                        Robot_Analyzers.Robot_ADC_CH5 = HID_report.Data[17] * 256 + HID_report.Data[18];
                        Robot_Analyzers.Robot_ADC_CH6 = HID_report.Data[19] * 256 + HID_report.Data[20];
                        Robot_Analyzers.Robot_ADC_CH7 = HID_report.Data[21] * 256 + HID_report.Data[22];
                        Robot_Analyzers.Robot_ADC_CH8 = HID_report.Data[23] * 256 + HID_report.Data[24];

                        Robot_Analyzers.EEprom_Read_Point = HID_report.Data[63];


                        if (Robot_Analyzers.EEprom_Mode == 0)
                        {
                            Robot_Analyzers.Robot_EXTI1 = HID_report.Data[25] * 256 + HID_report.Data[26];
                            Robot_Analyzers.Robot_EXTI2 = HID_report.Data[27] * 256 + HID_report.Data[28];
                            Robot_Analyzers.Robot_EXTI3 = HID_report.Data[29] * 256 + HID_report.Data[30];
                            Robot_Analyzers.Robot_EXTI4 = HID_report.Data[31] * 256 + HID_report.Data[32];
                            Robot_Analyzers.Robot_EXTI5 = HID_report.Data[33] * 256 + HID_report.Data[34];
                            //Robot_Analyzers.Robot_EXTI6 = HID_report.Data[35] * 256 + HID_report.Data[36];
                            //Robot_Analyzers.Robot_EXTI7 = HID_report.Data[37] * 256 + HID_report.Data[38];
                            //Robot_Analyzers.Robot_EXTI8 = HID_report.Data[39] * 256 + HID_report.Data[40];
                        }
                        else if (Robot_Analyzers.EEprom_Mode == 1)
                        {
                            if (Robot_Analyzers.EEprom_Read_Point == 0)
                            {
                                //Robot_Analyzers.EEprom_Read_Point = 1;
                                Robot_Analyzers.Robot_EEpromi[0] = (UInt16)(HID_report.Data[25] * 256 + HID_report.Data[26]);
                                Robot_Analyzers.Robot_EEpromi[1] = (UInt16)(HID_report.Data[27] * 256 + HID_report.Data[28]);
                                Robot_Analyzers.Robot_EEpromi[2] = (UInt16)(HID_report.Data[29] * 256 + HID_report.Data[30]);
                                Robot_Analyzers.Robot_EEpromi[3] = (UInt16)(HID_report.Data[31] * 256 + HID_report.Data[32]);
                                Robot_Analyzers.Robot_EEpromi[4] = (UInt16)(HID_report.Data[33] * 256 + HID_report.Data[34]);
                            }
                            else if (Robot_Analyzers.EEprom_Read_Point == 1)
                            {
                                //Robot_Analyzers.EEprom_Read_Point = 2;
                                Robot_Analyzers.Robot_EEpromi[5] = (UInt16)(HID_report.Data[25] * 256 + HID_report.Data[26]);
                                Robot_Analyzers.Robot_EEpromi[6] = (UInt16)(HID_report.Data[27] * 256 + HID_report.Data[28]);
                                Robot_Analyzers.Robot_EEpromi[7] = (UInt16)(HID_report.Data[29] * 256 + HID_report.Data[30]);
                                Robot_Analyzers.Robot_EEpromi[8] = (UInt16)(HID_report.Data[31] * 256 + HID_report.Data[32]);
                                Robot_Analyzers.Robot_EEpromi[9] = (UInt16)(HID_report.Data[33] * 256 + HID_report.Data[34]);
                            }
                            else if (Robot_Analyzers.EEprom_Read_Point == 2)
                            {
                                //Robot_Analyzers.EEprom_Read_Point = 3;
                                Robot_Analyzers.Robot_EEpromi[10] = (UInt16)(HID_report.Data[25] * 256 + HID_report.Data[26]);
                                Robot_Analyzers.Robot_EEpromi[11] = (UInt16)(HID_report.Data[27] * 256 + HID_report.Data[28]);
                                Robot_Analyzers.Robot_EEpromi[12] = (UInt16)(HID_report.Data[29] * 256 + HID_report.Data[30]);
                                Robot_Analyzers.Robot_EEpromi[13] = (UInt16)(HID_report.Data[31] * 256 + HID_report.Data[32]);
                                Robot_Analyzers.Robot_EEpromi[14] = (UInt16)(HID_report.Data[33] * 256 + HID_report.Data[34]);
                            }
                            else if (Robot_Analyzers.EEprom_Read_Point == 3)
                            {
                                //Robot_Analyzers.EEprom_Read_Point = 0;
                                Robot_Analyzers.Robot_EEpromi[15] = (UInt16)(HID_report.Data[25] * 256 + HID_report.Data[26]);
                                Robot_Analyzers.Robot_EEpromi[16] = (UInt16)(HID_report.Data[27] * 256 + HID_report.Data[28]);
                                Robot_Analyzers.Robot_EEpromi[17] = (UInt16)(HID_report.Data[29] * 256 + HID_report.Data[30]);
                                Robot_Analyzers.Robot_EEpromi[18] = (UInt16)(HID_report.Data[31] * 256 + HID_report.Data[32]);
                                Robot_Analyzers.Robot_EEpromi[19] = (UInt16)(HID_report.Data[33] * 256 + HID_report.Data[34]);

                             }
                        }

                        Robot_Analyzers.Robot_PES_Byte1 = HID_report.Data[43];
                        Robot_Analyzers.Robot_PES_Byte2 = HID_report.Data[44];
                        Robot_Analyzers.Robot_PES_Byte3 = HID_report.Data[45];
                        Robot_Analyzers.Robot_PES_Byte4 = HID_report.Data[46];
                        Robot_Analyzers.Robot_PES_Byte5 = HID_report.Data[47];
                        Robot_Analyzers.Robot_PES_Byte6 = HID_report.Data[48];
                        Robot_Analyzers.Robot_PES_Byte7 = HID_report.Data[49];
                        Robot_Analyzers.Robot_PES_Byte8 = HID_report.Data[50];

                        Robot_Analyzers.Robot_Encoder1 = 0;// HID_report.Data[53] * 256 + HID_report.Data[54];
                        Robot_Analyzers.Robot_Encoder2 = 0;// HID_report.Data[55] * 256 + HID_report.Data[56];

                        Robot_Analyzers.Compass_X_Reg = HID_report.Data[35] * 256 + HID_report.Data[36];
                        Robot_Analyzers.Compass_Y_Reg = HID_report.Data[37] * 256 + HID_report.Data[38];
                        Robot_Analyzers.Compass_Z_Reg = HID_report.Data[39] * 256 + HID_report.Data[40];

                        if (Robot_Analyzers.Compass_X_Reg > 32768) Robot_Analyzers.Compass_X_Reg = Robot_Analyzers.Compass_X_Reg - 65536;
                        if (Robot_Analyzers.Compass_Y_Reg > 32768) Robot_Analyzers.Compass_Y_Reg = Robot_Analyzers.Compass_Y_Reg - 65536;
                        if (Robot_Analyzers.Compass_Z_Reg > 32768) Robot_Analyzers.Compass_Z_Reg = Robot_Analyzers.Compass_Z_Reg - 65536;


                        Robot_Analyzers.Compass_X_Offset = (HID_report.Data[51] * 256 + HID_report.Data[52]) -32768;
                        Robot_Analyzers.Compass_Y_Offset = (HID_report.Data[53] * 256 + HID_report.Data[54]) -32768;
                        Robot_Analyzers.Compass_Z_Offset = (HID_report.Data[55] * 256 + HID_report.Data[56]) -32768;

                        Robot_Analyzers.Compass_X_Angle = (HID_report.Data[57] * 256 + HID_report.Data[58]) - 32768;
                        Robot_Analyzers.Compass_Y_Angle = (HID_report.Data[59] * 256 + HID_report.Data[60]) -32768;
                        Robot_Analyzers.Compass_Z_Angle = (HID_report.Data[61] * 256 + HID_report.Data[62]) - 32768;
                      
                        Robot_Analyzers.Compass_KZ_Angle = (HID_report.Data[41] * 256 + HID_report.Data[42]) - 32768;

                        ////Robot_Analyzers.Scale1 = Scale1;
                        ////Robot_Analyzers.Scale2 = Scale2;
                        ////Robot_Analyzers.Scale3 = Scale3;

                        Show_Val6 = Robot_Analyzers.Compass_Z_Angle;// (HID_report.Data[57] * 256 + HID_report.Data[58]) - 32768; 

                        if (Robot_Analyzers.Robot_View1 == 0) Show_Val1 = Robot_Analyzers.Robot_ADC_CH1;
                        else if (Robot_Analyzers.Robot_View1 == 1) Show_Val1 = Robot_Analyzers.Robot_ADC_CH2;
                        else if (Robot_Analyzers.Robot_View1 == 2) Show_Val1 = Robot_Analyzers.Robot_ADC_CH3;
                        else if (Robot_Analyzers.Robot_View1 == 3) Show_Val1 = Robot_Analyzers.Robot_ADC_CH4;
                        else if (Robot_Analyzers.Robot_View1 == 4) Show_Val1 = Robot_Analyzers.Robot_ADC_CH5;
                        else if (Robot_Analyzers.Robot_View1 == 5) Show_Val1 = Robot_Analyzers.Robot_ADC_CH6;
                        else if (Robot_Analyzers.Robot_View1 == 6) Show_Val1 = Robot_Analyzers.Robot_ADC_CH7;
                        else if (Robot_Analyzers.Robot_View1 == 7) Show_Val1 = Robot_Analyzers.Robot_ADC_CH8;
                        else if (Robot_Analyzers.Robot_View1 == 8) Show_Val1 = Robot_Analyzers.Compass_X_Reg;
                        else if (Robot_Analyzers.Robot_View1 == 9) Show_Val1 = Robot_Analyzers.Compass_Y_Reg;
                        else if (Robot_Analyzers.Robot_View1 == 10) Show_Val1 = Robot_Analyzers.Compass_Z_Reg;
                        else if (Robot_Analyzers.Robot_View1 == 11) Show_Val1 = Robot_Analyzers.Compass_X_Offset;
                        else if (Robot_Analyzers.Robot_View1 == 12) Show_Val1 = Robot_Analyzers.Compass_Y_Offset;
                        else if (Robot_Analyzers.Robot_View1 == 13) Show_Val1 = Robot_Analyzers.Compass_Z_Offset;
                        else if (Robot_Analyzers.Robot_View1 == 14) Show_Val1 = Robot_Analyzers.Compass_X_Angle;
                        else if (Robot_Analyzers.Robot_View1 == 15) Show_Val1 = Robot_Analyzers.Compass_Y_Angle;
                        else if (Robot_Analyzers.Robot_View1 == 16) Show_Val1 = Robot_Analyzers.Compass_Z_Angle;

                        if (Robot_Analyzers.Robot_View2 == 0) Show_Val2 = Robot_Analyzers.Robot_ADC_CH1;
                        else if (Robot_Analyzers.Robot_View2 == 1) Show_Val2 = Robot_Analyzers.Robot_ADC_CH2;
                        else if (Robot_Analyzers.Robot_View2 == 2) Show_Val2 = Robot_Analyzers.Robot_ADC_CH3;
                        else if (Robot_Analyzers.Robot_View2 == 3) Show_Val2 = Robot_Analyzers.Robot_ADC_CH4;
                        else if (Robot_Analyzers.Robot_View2 == 4) Show_Val2 = Robot_Analyzers.Robot_ADC_CH5;
                        else if (Robot_Analyzers.Robot_View2 == 5) Show_Val2 = Robot_Analyzers.Robot_ADC_CH6;
                        else if (Robot_Analyzers.Robot_View2 == 6) Show_Val2 = Robot_Analyzers.Robot_ADC_CH7;
                        else if (Robot_Analyzers.Robot_View2 == 7) Show_Val2 = Robot_Analyzers.Robot_ADC_CH8;
                        else if (Robot_Analyzers.Robot_View2 == 8) Show_Val2 = Robot_Analyzers.Compass_X_Reg;
                        else if (Robot_Analyzers.Robot_View2 == 9) Show_Val2 = Robot_Analyzers.Compass_Y_Reg;
                        else if (Robot_Analyzers.Robot_View2 == 10) Show_Val2 = Robot_Analyzers.Compass_Z_Reg;
                        else if (Robot_Analyzers.Robot_View2 == 11) Show_Val2 = Robot_Analyzers.Compass_X_Offset;
                        else if (Robot_Analyzers.Robot_View2 == 12) Show_Val2 = Robot_Analyzers.Compass_Y_Offset;
                        else if (Robot_Analyzers.Robot_View2 == 13) Show_Val2 = Robot_Analyzers.Compass_Z_Offset;
                        else if (Robot_Analyzers.Robot_View2 == 14) Show_Val2 = Robot_Analyzers.Compass_X_Angle;
                        else if (Robot_Analyzers.Robot_View2 == 15) Show_Val2 = Robot_Analyzers.Compass_Y_Angle;
                        else if (Robot_Analyzers.Robot_View2 == 16) Show_Val2 = Robot_Analyzers.Compass_Z_Angle;

                        if (Robot_Analyzers.Robot_View3 == 0) Show_Val3 = Robot_Analyzers.Robot_ADC_CH1;
                        else if (Robot_Analyzers.Robot_View3 == 1) Show_Val3 = Robot_Analyzers.Robot_ADC_CH2;
                        else if (Robot_Analyzers.Robot_View3 == 2) Show_Val3 = Robot_Analyzers.Robot_ADC_CH3;
                        else if (Robot_Analyzers.Robot_View3 == 3) Show_Val3 = Robot_Analyzers.Robot_ADC_CH4;
                        else if (Robot_Analyzers.Robot_View3 == 4) Show_Val3 = Robot_Analyzers.Robot_ADC_CH5;
                        else if (Robot_Analyzers.Robot_View3 == 5) Show_Val3 = Robot_Analyzers.Robot_ADC_CH6;
                        else if (Robot_Analyzers.Robot_View3 == 6) Show_Val3 = Robot_Analyzers.Robot_ADC_CH7;
                        else if (Robot_Analyzers.Robot_View3 == 7) Show_Val3 = Robot_Analyzers.Robot_ADC_CH8;
                        else if (Robot_Analyzers.Robot_View3 == 8) Show_Val3 = Robot_Analyzers.Compass_X_Reg;
                        else if (Robot_Analyzers.Robot_View3 == 9) Show_Val3 = Robot_Analyzers.Compass_Y_Reg;
                        else if (Robot_Analyzers.Robot_View3 == 10) Show_Val3 = Robot_Analyzers.Compass_Z_Reg;
                        else if (Robot_Analyzers.Robot_View3 == 11) Show_Val3 = Robot_Analyzers.Compass_X_Offset;
                        else if (Robot_Analyzers.Robot_View3 == 12) Show_Val3 = Robot_Analyzers.Compass_Y_Offset;
                        else if (Robot_Analyzers.Robot_View3 == 13) Show_Val3 = Robot_Analyzers.Compass_Z_Offset;
                        else if (Robot_Analyzers.Robot_View3 == 14) Show_Val3 = Robot_Analyzers.Compass_X_Angle;
                        else if (Robot_Analyzers.Robot_View3 == 15) Show_Val3 = Robot_Analyzers.Compass_Y_Angle;
                        else if (Robot_Analyzers.Robot_View3 == 16) Show_Val3 = Robot_Analyzers.Compass_Z_Angle;

                        if (Robot_Analyzers.Robot_View4 == 0) Show_Val4 = Robot_Analyzers.Robot_ADC_CH1;
                        else if (Robot_Analyzers.Robot_View4 == 1) Show_Val4 = Robot_Analyzers.Robot_ADC_CH2;
                        else if (Robot_Analyzers.Robot_View4 == 2) Show_Val4 = Robot_Analyzers.Robot_ADC_CH3;
                        else if (Robot_Analyzers.Robot_View4 == 3) Show_Val4 = Robot_Analyzers.Robot_ADC_CH4;
                        else if (Robot_Analyzers.Robot_View4 == 4) Show_Val4 = Robot_Analyzers.Robot_ADC_CH5;
                        else if (Robot_Analyzers.Robot_View4 == 5) Show_Val4 = Robot_Analyzers.Robot_ADC_CH6;
                        else if (Robot_Analyzers.Robot_View4 == 6) Show_Val4 = Robot_Analyzers.Robot_ADC_CH7;
                        else if (Robot_Analyzers.Robot_View4 == 7) Show_Val4 = Robot_Analyzers.Robot_ADC_CH8;
                        else if (Robot_Analyzers.Robot_View4 == 8) Show_Val4 = Robot_Analyzers.Compass_X_Reg;
                        else if (Robot_Analyzers.Robot_View4 == 9) Show_Val4 = Robot_Analyzers.Compass_Y_Reg;
                        else if (Robot_Analyzers.Robot_View4 == 10) Show_Val4 = Robot_Analyzers.Compass_Z_Reg;
                        else if (Robot_Analyzers.Robot_View4 == 11) Show_Val4 = Robot_Analyzers.Compass_X_Offset;
                        else if (Robot_Analyzers.Robot_View4 == 12) Show_Val4 = Robot_Analyzers.Compass_Y_Offset;
                        else if (Robot_Analyzers.Robot_View4 == 13) Show_Val4 = Robot_Analyzers.Compass_Z_Offset;
                        else if (Robot_Analyzers.Robot_View4 == 14) Show_Val4 = Robot_Analyzers.Compass_X_Angle;
                        else if (Robot_Analyzers.Robot_View4 == 15) Show_Val4 = Robot_Analyzers.Compass_Y_Angle;
                        else if (Robot_Analyzers.Robot_View4 == 16) Show_Val4 = Robot_Analyzers.Compass_Z_Angle;

                        Show_Counter++;

                        if (Show_Counter >= 5)
                        {
                            Show_Counter = 0;
                            if (Robot_Analyzers.EEprom_Mode == 0)
                            {
                                RB_Show_S1.Dispatcher.Invoke(() => RB_Show_S1.Text = string.Concat(Robot_Analyzers.Robot_GPIO1.ToString("X2")));
                                RB_Show_S2.Dispatcher.Invoke(() => RB_Show_S2.Text = string.Concat(Robot_Analyzers.Robot_GPIO2.ToString("X2")));
                                RB_Show_S3.Dispatcher.Invoke(() => RB_Show_S3.Text = string.Concat(Robot_Analyzers.Robot_GPIO3.ToString("X2")));
                                RB_Show_S4.Dispatcher.Invoke(() => RB_Show_S4.Text = string.Concat(Robot_Analyzers.Robot_GPIO4.ToString("X2")));
                                RB_Show_S5.Dispatcher.Invoke(() => RB_Show_S5.Text = string.Concat(Robot_Analyzers.Robot_GPIO5.ToString("X2")));
                                RB_Show_S6.Dispatcher.Invoke(() => RB_Show_S6.Text = string.Concat(Robot_Analyzers.Robot_GPIO6.ToString("X2")));
                                RB_Show_S61.Dispatcher.Invoke(() => RB_Show_S61.Text = string.Concat(Robot_Analyzers.Robot_GPIO7.ToString("X2")));

                                RB_Show_S7.Dispatcher.Invoke(() => RB_Show_S7.Text = string.Concat(Robot_Analyzers.Robot_ADC_CH1));
                                RB_Show_S8.Dispatcher.Invoke(() => RB_Show_S8.Text = string.Concat(Robot_Analyzers.Robot_ADC_CH2));
                                RB_Show_S9.Dispatcher.Invoke(() => RB_Show_S9.Text = string.Concat(Robot_Analyzers.Robot_ADC_CH3));
                                RB_Show_S10.Dispatcher.Invoke(() => RB_Show_S10.Text = string.Concat(Robot_Analyzers.Robot_ADC_CH4));
                                RB_Show_S11.Dispatcher.Invoke(() => RB_Show_S11.Text = string.Concat(Robot_Analyzers.Robot_ADC_CH5));
                                RB_Show_S12.Dispatcher.Invoke(() => RB_Show_S12.Text = string.Concat(Robot_Analyzers.Robot_ADC_CH6));
                                RB_Show_S13.Dispatcher.Invoke(() => RB_Show_S13.Text = string.Concat(Robot_Analyzers.Robot_ADC_CH7));
                                RB_Show_S14.Dispatcher.Invoke(() => RB_Show_S14.Text = string.Concat(Robot_Analyzers.Compass_X_Reg));

                                RB_Show_S15.Dispatcher.Invoke(() => RB_Show_S15.Text = string.Concat(Robot_Analyzers.Compass_Y_Reg));
                                RB_Show_S16.Dispatcher.Invoke(() => RB_Show_S16.Text = string.Concat(Robot_Analyzers.Compass_Z_Reg));

                                RB_Show_S17.Dispatcher.Invoke(() => RB_Show_S17.Text = string.Concat(Robot_Analyzers.Robot_EXTI1));
                                RB_Show_S18.Dispatcher.Invoke(() => RB_Show_S18.Text = string.Concat(Robot_Analyzers.Robot_EXTI2));
                                RB_Show_S19.Dispatcher.Invoke(() => RB_Show_S19.Text = string.Concat(Robot_Analyzers.Robot_EXTI3));
                                RB_Show_S20.Dispatcher.Invoke(() => RB_Show_S20.Text = string.Concat(Robot_Analyzers.Compass_X_Offset));
                                //RB_Show_S21.Dispatcher.Invoke(() => RB_Show_S21.Text = string.Concat(Robot_Analyzers.Robot_EXTI5));
                                RB_Show_S22.Dispatcher.Invoke(() => RB_Show_S22.Text = string.Concat(Robot_Analyzers.Compass_Y_Offset));
                                //RB_Show_S23.Dispatcher.Invoke(() => RB_Show_S23.Text = string.Concat(Robot_Analyzers.Robot_EXTI7));
                                RB_Show_S24.Dispatcher.Invoke(() => RB_Show_S24.Text = string.Concat(Robot_Analyzers.Compass_Z_Offset));

                                RB_Show_PE1.Dispatcher.Invoke(() => RB_Show_PE1.Text = string.Concat(Robot_Analyzers.Robot_PES_Byte1.ToString("X2")));
                                RB_Show_PE2.Dispatcher.Invoke(() => RB_Show_PE2.Text = string.Concat(Robot_Analyzers.Robot_PES_Byte2.ToString("X2")));
                                RB_Show_PE3.Dispatcher.Invoke(() => RB_Show_PE3.Text = string.Concat(Robot_Analyzers.Robot_PES_Byte3.ToString("X2")));
                                RB_Show_PE4.Dispatcher.Invoke(() => RB_Show_PE4.Text = string.Concat(Robot_Analyzers.Robot_PES_Byte4.ToString("X2")));
                                RB_Show_PE5.Dispatcher.Invoke(() => RB_Show_PE5.Text = string.Concat(Robot_Analyzers.Robot_PES_Byte5.ToString("X2")));
                                RB_Show_PE6.Dispatcher.Invoke(() => RB_Show_PE6.Text = string.Concat(Robot_Analyzers.Robot_PES_Byte6.ToString("X2")));
                                RB_Show_PE7.Dispatcher.Invoke(() => RB_Show_PE7.Text = string.Concat(Robot_Analyzers.Robot_PES_Byte7.ToString("X2")));
                                RB_Show_PE8.Dispatcher.Invoke(() => RB_Show_PE8.Text = string.Concat(Robot_Analyzers.Robot_PES_Byte8.ToString("X2")));


                                Y_SCopeu.Dispatcher.Invoke(() => Y_SCopeu.VisibleRange = new DoubleRange(Show_Val3 - Robot_Analyzers.Scale1, Show_Val3 + Robot_Analyzers.Scale1));
                                Y_SCopeu1.Dispatcher.Invoke(() => Y_SCopeu1.VisibleRange = new DoubleRange(Show_Val2 - Robot_Analyzers.Scale2, Show_Val2 + Robot_Analyzers.Scale2));
                                Y_SCopeu2.Dispatcher.Invoke(() => Y_SCopeu2.VisibleRange = new DoubleRange(Show_Val6 - Robot_Analyzers.Scale3, Show_Val6 + Robot_Analyzers.Scale3));

                                RB_Show_compass1.Dispatcher.Invoke(() => RB_Show_compass1.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_X_Angle / 10.0, 4), " độ"));
                                RB_Show_compass2.Dispatcher.Invoke(() => RB_Show_compass2.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_Y_Angle / 10.0, 4), " độ"));
                                RB_Show_compass3.Dispatcher.Invoke(() => RB_Show_compass3.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_Z_Angle / 10.0, 4), " độ"));
                                RB_Show_compass4.Dispatcher.Invoke(() => RB_Show_compass4.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_KZ_Angle / 10.0, 4), " độ"));
                                //Show_Val5 = Show_Val4 + Show_Val2 - 32768;
                            }
                        else if (Robot_Analyzers.EEprom_Mode == 1)
                            {
                                RB_SVF1.Dispatcher.Invoke(() => RB_SVF1.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[0]));// = Convert.ToUInt16(RB_SV1.Text);
                                RB_SVF2.Dispatcher.Invoke(() => RB_SVF2.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[1]));//  = Convert.ToUInt16(RB_SV2.Text);
                                RB_SVF3.Dispatcher.Invoke(() => RB_SVF3.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[2]));//  = Convert.ToUInt16(RB_SV3.Text);
                                RB_SVF4.Dispatcher.Invoke(() => RB_SVF4.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[3]));//  = Convert.ToUInt16(RB_SV4.Text);
                                RB_SVF5.Dispatcher.Invoke(() => RB_SVF5.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[4]));//  = Convert.ToUInt16(RB_SV5.Text);
                                RB_SVF6.Dispatcher.Invoke(() => RB_SVF6.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[5]));//  = Convert.ToUInt16(RB_SV6.Text);
                                RB_SVF7.Dispatcher.Invoke(() => RB_SVF7.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[6]));//  = Convert.ToUInt16(RB_SV7.Text);
                                RB_SVF8.Dispatcher.Invoke(() => RB_SVF8.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[7]));//  = Convert.ToUInt16(RB_SV8.Text);
                                RB_SVF9.Dispatcher.Invoke(() => RB_SVF9.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[8]));//  = Convert.ToUInt16(RB_SV9.Text);
                                RB_SVF10.Dispatcher.Invoke(() => RB_SVF10.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[9]));//  = Convert.ToUInt16(RB_SV10.Text);
                                RB_SVF11.Dispatcher.Invoke(() => RB_SVF11.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[10]));//  = Convert.ToUInt16(RB_SV11.Text);
                                RB_SVF12.Dispatcher.Invoke(() => RB_SVF12.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[11]));//  = Convert.ToUInt16(RB_SV12.Text);
                                RB_SVF13.Dispatcher.Invoke(() => RB_SVF13.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[12]));//  = Convert.ToUInt16(RB_SV13.Text);
                                RB_SVF14.Dispatcher.Invoke(() => RB_SVF14.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[13]));//  = Convert.ToUInt16(RB_SV14.Text);
                                RB_SVF15.Dispatcher.Invoke(() => RB_SVF15.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[14]));//  = Convert.ToUInt16(RB_SV15.Text);
                                RB_SVF16.Dispatcher.Invoke(() => RB_SVF16.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[15]));//  = Convert.ToUInt16(RB_SV16.Text);
                                RB_SVF17.Dispatcher.Invoke(() => RB_SVF17.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[16]));//  = Convert.ToUInt16(RB_SV17.Text);
                                RB_SVF18.Dispatcher.Invoke(() => RB_SVF18.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[17]));//  = Convert.ToUInt16(RB_SV18.Text);
                                RB_SVF19.Dispatcher.Invoke(() => RB_SVF19.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[18]));//  = Convert.ToUInt16(RB_SV19.Text);
                                RB_SVF20.Dispatcher.Invoke(() => RB_SVF20.Text = string.Concat(Robot_Analyzers.Robot_EEpromi[19]));//  = Convert.ToUInt16(RB_SV20.Text);
                            
                                if(Robot_Analyzers.EEprom_Load == 1)
                                {
                                    Robot_Analyzers.EEprom_Load = 0;
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text = string.Concat("Data[00] = ", Robot_Analyzers.Robot_EEpromi[0], "\r\n"));// = Convert.ToUInt16(RB_SV1.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[01] = ", Robot_Analyzers.Robot_EEpromi[1], "\r\n"));//  = Convert.ToUInt16(RB_SV2.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[02] = ", Robot_Analyzers.Robot_EEpromi[2], "\r\n"));//  = Convert.ToUInt16(RB_SV3.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[03] = ", Robot_Analyzers.Robot_EEpromi[3], "\r\n"));//  = Convert.ToUInt16(RB_SV4.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[04] = ", Robot_Analyzers.Robot_EEpromi[4], "\r\n"));//  = Convert.ToUInt16(RB_SV5.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[05] = ", Robot_Analyzers.Robot_EEpromi[5], "\r\n"));//  = Convert.ToUInt16(RB_SV6.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[06] = ", Robot_Analyzers.Robot_EEpromi[6], "\r\n"));//  = Convert.ToUInt16(RB_SV7.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[07] = ", Robot_Analyzers.Robot_EEpromi[7], "\r\n"));//  = Convert.ToUInt16(RB_SV8.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[08] = ", Robot_Analyzers.Robot_EEpromi[8], "\r\n"));//  = Convert.ToUInt16(RB_SV9.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[09] = ", Robot_Analyzers.Robot_EEpromi[9], "\r\n"));//  = Convert.ToUInt16(RB_SV10.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[10] = ", Robot_Analyzers.Robot_EEpromi[10], "\r\n"));//  = Convert.ToUInt16(RB_SV11.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[11] = ", Robot_Analyzers.Robot_EEpromi[11], "\r\n"));//  = Convert.ToUInt16(RB_SV12.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[12] = ", Robot_Analyzers.Robot_EEpromi[12], "\r\n"));//  = Convert.ToUInt16(RB_SV13.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[13] = ", Robot_Analyzers.Robot_EEpromi[13], "\r\n"));//  = Convert.ToUInt16(RB_SV14.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[14] = ", Robot_Analyzers.Robot_EEpromi[14], "\r\n"));//  = Convert.ToUInt16(RB_SV15.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[15] = ", Robot_Analyzers.Robot_EEpromi[15], "\r\n"));//  = Convert.ToUInt16(RB_SV16.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[16] = ", Robot_Analyzers.Robot_EEpromi[16], "\r\n"));//  = Convert.ToUInt16(RB_SV17.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[17] = ", Robot_Analyzers.Robot_EEpromi[17], "\r\n"));//  = Convert.ToUInt16(RB_SV18.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[18] = ", Robot_Analyzers.Robot_EEpromi[18], "\r\n"));//  = Convert.ToUInt16(RB_SV19.Text);
                                    RB_SV20_LOG.Dispatcher.Invoke(() => RB_SV20_LOG.Text += string.Concat("Data[19] = ", Robot_Analyzers.Robot_EEpromi[19], "\r\n"));//  = Convert.ToUInt16(RB_SV20.Text);

                                }
                            }

                            RB_Show_counter.Dispatcher.Invoke(() => RB_Show_counter.Text = string.Concat(Robot_Analyzers.Robot_Counter));

                            if (status == 0)
                            {
                                status = 1;
                                usb_Status.Dispatcher.Invoke(() => usb_Status.Fill = System.Windows.Media.Brushes.Green);
                            }
                            else if (status == 1)
                            {
                                status = 0;
                                usb_Status.Dispatcher.Invoke(() => usb_Status.Fill = System.Windows.Media.Brushes.Red);
                            }

                        }

                        if (Robot_Analyzers.EEprom_Mode == 0)
                        {

                            try
                            {
                                Robot_Analyzers._xyDataSeries_Robot1.Append(Robot_Analyzers.Robot_Counter, Show_Val1);
                                Robot_Analyzers._xyDataSeries_Robot2.Append(Robot_Analyzers.Robot_Counter, Show_Val2);
                                Robot_Analyzers._xyDataSeries_Robot12.Append(Robot_Analyzers.Robot_Counter, Show_Val3);
                                Robot_Analyzers._xyDataSeries_Robot22.Append(Robot_Analyzers.Robot_Counter, Show_Val4);
                                Robot_Analyzers._xyDataSeries_Robot3.Append(Robot_Analyzers.Robot_Counter, Show_Val6);
                                Robot_Analyzers._xyDataSeries_Robot4.Append(Robot_Analyzers.Robot_Counter, Robot_Analyzers.Compass_KZ_Angle);
                            }
                            catch { }

                            if (Robot_Analyzers.Robot_Counter > 20000) Robot_Analyzers.Robot_Clear = 1;

                            if ((Robot_Analyzers.Robot_Counter >= 150) && (Robot_Analyzers.Robot_Counter % 150 == 0))
                            {
                                X_SCopeu.Dispatcher.Invoke(() => X_SCopeu.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robot_Counter + 150));
                                X_SCopeu1.Dispatcher.Invoke(() => X_SCopeu1.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robot_Counter + 150));
                                X_SCopeu2.Dispatcher.Invoke(() => X_SCopeu2.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robot_Counter + 150));
                            }

                            Robot_Analyzers.Robot_Counter++;

                            if (Robot_Analyzers.Robot_Clear == 1)
                            {
                                Robot_Analyzers.Robot_Clear = 0;
                                Robot_Analyzers.Robot_Counter= 0;

                                X_SCopeu.Dispatcher.Invoke(() => X_SCopeu.VisibleRange = new DoubleRange(0, 150));
                                X_SCopeu1.Dispatcher.Invoke(() => X_SCopeu1.VisibleRange = new DoubleRange(0, 150));
                                X_SCopeu2.Dispatcher.Invoke(() => X_SCopeu2.VisibleRange = new DoubleRange(0, 150));

                                U1_Line1i.Dispatcher.Invoke(() => U1_Line1i.X1 = 30);
                                U1_Line1i1.Dispatcher.Invoke(() => U1_Line1i1.X1 = 30);
                                U1_Line1i2.Dispatcher.Invoke(() => U1_Line1i2.X1 = 30);


                                //_xyDataSeries_rba = new XyDataSeries<double>();
                                Robot_Analyzers._xyDataSeries_Robot1 = new XyDataSeries<double>();
                                Robot_Analyzers._xyDataSeries_Robot2 = new XyDataSeries<double>();
                                Robot_Analyzers._xyDataSeries_Robot3 = new XyDataSeries<double>();
                                Robot_Analyzers._xyDataSeries_Robot4 = new XyDataSeries<double>();
                                Robot_Analyzers._xyDataSeries_Robot12 = new XyDataSeries<double>();
                                Robot_Analyzers._xyDataSeries_Robot22 = new XyDataSeries<double>();

                                //Sensor_Compass.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = _xyDataSeries_rba);
                                Sensor_U1.Dispatcher.Invoke(() => Sensor_U1.DataSeries = Robot_Analyzers._xyDataSeries_Robot1);
                                Sensor_U2.Dispatcher.Invoke(() => Sensor_U2.DataSeries = Robot_Analyzers._xyDataSeries_Robot2);
                                Sensor_U3.Dispatcher.Invoke(() => Sensor_U3.DataSeries = Robot_Analyzers._xyDataSeries_Robot3);
                                Sensor_U4.Dispatcher.Invoke(() => Sensor_U4.DataSeries = Robot_Analyzers._xyDataSeries_Robot4);
                                Sensor_U12.Dispatcher.Invoke(() => Sensor_U12.DataSeries = Robot_Analyzers._xyDataSeries_Robot12);
                                Sensor_U22.Dispatcher.Invoke(() => Sensor_U22.DataSeries = Robot_Analyzers._xyDataSeries_Robot22);

                                //Robot_Analyzers.Robo_Sample = 0;
                                //Robot_Analyzers.Max_ADC = 100;
                                //Robot_Analyzers.Max_Compass = 10;
                                //Robot_Analyzers.Min_Compass = -10;
                                //Robot_Analyzers.Max_ABC = 100;
                                //Robot_Analyzers.Max_ADCi = 100;
                                //Robot_Analyzers.Max_Compassi = 10;
                                //Robot_Analyzers.Min_Compassi = -10;
                                //Robot_Analyzers.Max_ABCi = 100;
                                //Robot_Analyzers.Compass_input3_Maxi = 1.2;
                                //Robot_Analyzers.Robot_Point = 0;

                                //Y_SCope3.Dispatcher.Invoke(() => Y_SCope3.VisibleRange = new DoubleRange(0, 100));
                                //Y_SCope5.Dispatcher.Invoke(() => Y_SCope5.VisibleRange = new DoubleRange(-10, 10));
                                //Y_SCope6.Dispatcher.Invoke(() => Y_SCope6.VisibleRange = new DoubleRange(0, 100));
                                //Y_SCope7.Dispatcher.Invoke(() => Y_SCope7.VisibleRange = new DoubleRange(0, 1.2));
                            }


                        }


                    }
                }

            }
        }

        private void ReadReportCallback(HidReport report)
        {
            HID_report = report;
            PID_struct.HID_Done = 1;
            try
            {
                if (device.IsConnected == true) device.ReadReport(ReadReportCallback);
            }
            catch
            {

            }
        }


        #endregion

        #region process thread

        public void Robot_Clear()
        {
            X_SCope4.Dispatcher.Invoke(() => X_SCope4.VisibleRange = new DoubleRange(0, 150));
            X_SCope3.Dispatcher.Invoke(() => X_SCope3.VisibleRange = new DoubleRange(0, 150));
            X_SCope5.Dispatcher.Invoke(() => X_SCope5.VisibleRange = new DoubleRange(0, 150));
            X_SCope6.Dispatcher.Invoke(() => X_SCope6.VisibleRange = new DoubleRange(0, 150));
            X_SCope7.Dispatcher.Invoke(() => X_SCope7.VisibleRange = new DoubleRange(0, 150));

            S1_Line1i.Dispatcher.Invoke(() => S1_Line1i.X1 = 30);
            S2_Line1i.Dispatcher.Invoke(() => S2_Line1i.X1 = 30);
            S3_Line1i.Dispatcher.Invoke(() => S3_Line1i.X1 = 30);
            S4_Line1i.Dispatcher.Invoke(() => S4_Line1i.X1 = 30);
            S5_Line1i.Dispatcher.Invoke(() => S5_Line1i.X1 = 30);

            //_xyDataSeries_rba = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb2 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb3 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb4 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb5 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb6 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb7 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb8 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb9 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb10 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb11 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb12 = new XyDataSeries<double>();
            Robot_Analyzers._xyDataSeries_rb13 = new XyDataSeries<double>();

            Robot_Analyzers._xyDataSeries_IO9 = new XyDataSeries<double>();

            //Sensor_Compass.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = _xyDataSeries_rba);
            Sensor_Duty1.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = Robot_Analyzers._xyDataSeries_rb2);
            Sensor_Duty2.Dispatcher.Invoke(() => Sensor_Duty2.DataSeries = Robot_Analyzers._xyDataSeries_rb3);
            Sensor_Duty3.Dispatcher.Invoke(() => Sensor_Duty3.DataSeries = Robot_Analyzers._xyDataSeries_rb4);
            Sensor_Duty4.Dispatcher.Invoke(() => Sensor_Duty4.DataSeries = Robot_Analyzers._xyDataSeries_rb5);

            Sensor_adcData1.Dispatcher.Invoke(() => Sensor_adcData1.DataSeries = Robot_Analyzers._xyDataSeries_rb6);
            Sensor_adcData2.Dispatcher.Invoke(() => Sensor_adcData2.DataSeries = Robot_Analyzers._xyDataSeries_rb7);
            Sensor_adcData3.Dispatcher.Invoke(() => Sensor_adcData3.DataSeries = Robot_Analyzers._xyDataSeries_rb8);

            Sensor_compassData.Dispatcher.Invoke(() => Sensor_compassData.DataSeries = Robot_Analyzers._xyDataSeries_rb9);

            Sensor_A_Data.Dispatcher.Invoke(() => Sensor_A_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb10);
            Sensor_B_Data.Dispatcher.Invoke(() => Sensor_B_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb11);
            Sensor_C_Data.Dispatcher.Invoke(() => Sensor_C_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb12);
            Sensor_Encoder.Dispatcher.Invoke(() => Sensor_Encoder.DataSeries = Robot_Analyzers._xyDataSeries_rb13);

            Sensor_lOGIC9.Dispatcher.Invoke(() => Sensor_lOGIC9.DataSeries = Robot_Analyzers._xyDataSeries_IO9);

            Robot_Analyzers.Robo_Sample = 0;
            Robot_Analyzers.Max_ADC = 100;
            Robot_Analyzers.Max_Compass = 10;
            Robot_Analyzers.Min_Compass = -10;
            Robot_Analyzers.Max_ABC = 100;
            Robot_Analyzers.Max_ADCi = 100;
            Robot_Analyzers.Max_Compassi = 10;
            Robot_Analyzers.Min_Compassi = -10;
            Robot_Analyzers.Max_ABCi = 100;
            Robot_Analyzers.Compass_input3_Maxi = 1.2;
            Robot_Analyzers.Robot_Point = 0;

            Y_SCope3.Dispatcher.Invoke(() => Y_SCope3.VisibleRange = new DoubleRange(0, 100));
            Y_SCope5.Dispatcher.Invoke(() => Y_SCope5.VisibleRange = new DoubleRange(-10, 10));
            Y_SCope6.Dispatcher.Invoke(() => Y_SCope6.VisibleRange = new DoubleRange(0, 100));
            Y_SCope7.Dispatcher.Invoke(() => Y_SCope7.VisibleRange = new DoubleRange(0, 1.2));
        }

        public void Save_Packet1()
        {
            if (Robot_Analyzers.Robot_Packet_Save_File == 1)
            {
                Robot_Analyzers.Robot_Packet_Save_File = 0;
                Array.Copy(Robot_Analyzers.Robo_Frame, 0, Robot_Analyzers.Robot_Packet_Frame, Robot_Analyzers.Robot_Packet_Count * 32, 32);
                // tăng số đếm
                if (Robot_Analyzers.Robot_Packet_Count < 19999) Robot_Analyzers.Robot_Packet_Count++;
                //label_Progess.Dispatcher.Invoke(() => label_Progess.Content = string.Concat("S= ", PID_struct.PID_Packet_Count, "  T= ", Math.Round(PID_struct.PID_Packet_Count / 40.0, 2), "s"));
            }

            if (Robot_Analyzers.Robot_Packet_Save_File == 2)
            {
                Robot_Analyzers.Robot_Packet_Save_File = 0;

                string path = "";
                string Config_Dir = System.AppDomain.CurrentDomain.BaseDirectory;

                string times = DateTime.Now.ToString("hh_mm_ss_dd_MM_yyyy");
                string times2 = DateTime.Now.ToString("dd_MM_yyyy");
                string Save_DIR = String.Concat(Config_Dir, "/Voi_", times2);

                path = String.Concat(Save_DIR, "/Robot_Voi__", times, ".Robot");
                if (!System.IO.Directory.Exists(Save_DIR)) System.IO.Directory.CreateDirectory(Save_DIR);

                int File_Size = Robot_Analyzers.Robot_Packet_Count * 32;

                File.WriteAllBytes(path, Robot_Analyzers.Robot_Packet_Frame);

                if (Directory.Exists(Save_DIR)) //  ADC_GPR.GSF.GSF_Dir1
                {
                    try
                    {
                        // Open file for reading
                        System.IO.FileStream _FileStream = new System.IO.FileStream(path, System.IO.FileMode.Create, System.IO.FileAccess.Write);
                        // Writes a block of bytes to this stream using data from
                        // a byte array.
                        _FileStream.Write(Robot_Analyzers.Robot_Packet_Frame, 0, File_Size);

                        // close file stream
                        _FileStream.Close();

                        System.Diagnostics.Process.Start(Save_DIR);
                    }

                    catch (Exception _Exception)
                    {
                        // Error

                    }
                }
                else
                {
                    //System.Windows.MessageBox.Show(string.Concat("Đường dẫn lưu file không tồn tại!\n Cài đặt lại đường dẫn file"));
                }

            }
        }

        public void Save_Packet2()
        {
            if (Robot_Analyzers.Robot_Packet_Save_File == 1)
            {
                Array.Copy(Robot_Analyzers.Robo_Frame2, 0, Robot_Analyzers.Robot_Packet_Frame, Robot_Analyzers.Robot_Packet_Count * 32, 32);
                // tăng số đếm
                if (Robot_Analyzers.Robot_Packet_Count < 19999) Robot_Analyzers.Robot_Packet_Count++;
                //label_Progess.Dispatcher.Invoke(() => label_Progess.Content = string.Concat("S= ", PID_struct.PID_Packet_Count, "  T= ", Math.Round(PID_struct.PID_Packet_Count / 40.0, 2), "s"));
            }

            if (Robot_Analyzers.Robot_Packet_Save_File == 2)
            {
                Robot_Analyzers.Robot_Packet_Save_File = 0;

                string path = "";
                string Config_Dir = System.AppDomain.CurrentDomain.BaseDirectory;

                string times = DateTime.Now.ToString("hh_mm_ss_dd_MM_yyyy");
                string times2 = DateTime.Now.ToString("dd_MM_yyyy");
                string Save_DIR = String.Concat(Config_Dir, "/Tho_", times2);

                path = String.Concat(Save_DIR, "/Robot_Tho__", times, ".Robot");
                if (!System.IO.Directory.Exists(Save_DIR)) System.IO.Directory.CreateDirectory(Save_DIR);

                int File_Size = Robot_Analyzers.Robot_Packet_Count * 32;

                File.WriteAllBytes(path, Robot_Analyzers.Robot_Packet_Frame);

                if (Directory.Exists(Save_DIR)) //  ADC_GPR.GSF.GSF_Dir1
                {
                    try
                    {
                        // Open file for reading
                        System.IO.FileStream _FileStream = new System.IO.FileStream(path, System.IO.FileMode.Create, System.IO.FileAccess.Write);
                        // Writes a block of bytes to this stream using data from
                        // a byte array.
                        _FileStream.Write(Robot_Analyzers.Robot_Packet_Frame, 0, File_Size);

                        // close file stream
                        _FileStream.Close();

                        System.Diagnostics.Process.Start(Save_DIR);
                    }

                    catch (Exception _Exception)
                    {
                        // Error

                    }
                }
                else
                {
                    //System.Windows.MessageBox.Show(string.Concat("Đường dẫn lưu file không tồn tại!\n Cài đặt lại đường dẫn file"));
                }

            }
        }

        public void Analyse_FEE_HAUI()
        {
            string Innit_dirrr = System.AppDomain.CurrentDomain.BaseDirectory;
            openFileDialog1.InitialDirectory = Innit_dirrr;
            openFileDialog1.Filter = "Robot file (*.Robot)|*.robot";
            openFileDialog1.RestoreDirectory = true;

            if (openFileDialog1.ShowDialog() == true)
            {
                long File_Size = new FileInfo(openFileDialog1.FileName).Length;
                string FEE_HAUI_File_DIR = openFileDialog1.FileName;
                string FEE_HAUI_Folder = System.IO.Path.GetDirectoryName(openFileDialog1.FileName);
                string FEE_HAUI_File_Folder = System.IO.Path.GetDirectoryName(openFileDialog1.FileName);

                if (File_Size % 32 == 0)
                {
                    Robot_Analyzers.Robot_Packet_Save = (int)(File_Size / 32);
                    //tbFile_Size.Text = String.Concat("fileSize = ", File_Size, "  Line = ", PID_struct.PID_Packet_size);

                    byte[] Buff_Data = new byte[File_Size];
                    byte[] Buff_Datai = new byte[32];

                    using (MemoryMappedFile memoryMappedFile = MemoryMappedFile.CreateFromFile(FEE_HAUI_File_DIR, FileMode.Open))
                    {
                        using (MemoryMappedViewStream memoryMappedViewStream = memoryMappedFile.CreateViewStream(0, File_Size))
                        {
                            memoryMappedViewStream.Read(Buff_Data, 0, (int)(File_Size));
                        }
                    }

                    Robot_Analyzers.connect = 0;
                    Robot_Analyzers.Counters = 0;
                    Robot_Analyzers.Counters2 = 0;

                    Robot_Analyzers.Max_ADC = 0;
                    Robot_Analyzers.Max_Compass = 0;
                    Robot_Analyzers.Min_Compass = 0;
                    Robot_Analyzers.Max_ABC = 100;

                    Robot_Analyzers.Max_ADCi = 0;
                    Robot_Analyzers.Max_Compassi = 0;
                    Robot_Analyzers.Min_Compassi = 0;
                    Robot_Analyzers.Max_ABCi = 100;

                    //Sensor_Compass.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = _xyDataSeries_rba);
                    Sensor_Duty1.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = Robot_Analyzers._xyDataSeries_rb2);
                    Sensor_Duty2.Dispatcher.Invoke(() => Sensor_Duty2.DataSeries = Robot_Analyzers._xyDataSeries_rb3);
                    Sensor_Duty3.Dispatcher.Invoke(() => Sensor_Duty3.DataSeries = Robot_Analyzers._xyDataSeries_rb4);
                    Sensor_Duty4.Dispatcher.Invoke(() => Sensor_Duty4.DataSeries = Robot_Analyzers._xyDataSeries_rb5);

                    Sensor_adcData1.Dispatcher.Invoke(() => Sensor_adcData1.DataSeries = Robot_Analyzers._xyDataSeries_rb6);
                    Sensor_adcData2.Dispatcher.Invoke(() => Sensor_adcData2.DataSeries = Robot_Analyzers._xyDataSeries_rb7);
                    Sensor_adcData3.Dispatcher.Invoke(() => Sensor_adcData3.DataSeries = Robot_Analyzers._xyDataSeries_rb8);
                    Sensor_Encoder.Dispatcher.Invoke(() => Sensor_Encoder.DataSeries = Robot_Analyzers._xyDataSeries_rb13);

                    Sensor_compassData.Dispatcher.Invoke(() => Sensor_compassData.DataSeries = Robot_Analyzers._xyDataSeries_rb9);

                    Sensor_A_Data.Dispatcher.Invoke(() => Sensor_A_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb10);
                    Sensor_B_Data.Dispatcher.Invoke(() => Sensor_B_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb11);
                    Sensor_C_Data.Dispatcher.Invoke(() => Sensor_C_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb12);

                    Sensor_lOGIC9.Dispatcher.Invoke(() => Sensor_lOGIC9.DataSeries = Robot_Analyzers._xyDataSeries_IO9);

                    Robot_Analyzers.Robot_View_Delay = Buff_Data[10];

                    RB_tbx.Dispatcher.Invoke(() => RB_tbx.Text = string.Concat(Robot_Analyzers.Robot_Packet_Save));
                    RB_tbx_time.Dispatcher.Invoke(() => 
                    RB_tbx_time.Text = string.Concat(Math.Round((Robot_Analyzers.Robot_Packet_Save * Robot_Analyzers.Robot_View_Delay) / 1000.0, 2), "s"));

                    X_SCope4.Dispatcher.Invoke(() => X_SCope4.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robot_Packet_Save));
                    X_SCope3.Dispatcher.Invoke(() => X_SCope3.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robot_Packet_Save));
                    X_SCope5.Dispatcher.Invoke(() => X_SCope5.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robot_Packet_Save));
                    X_SCope6.Dispatcher.Invoke(() => X_SCope6.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robot_Packet_Save));
                    X_SCope7.Dispatcher.Invoke(() => X_SCope7.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robot_Packet_Save));

                    for (int i=0; i< Robot_Analyzers.Robot_Packet_Save; i++) 
                    {
                        Robot_Analyzers.Robo_Restart = Buff_Data[i * 32 + 3];
                        Robot_Analyzers.Robo_Encoder = (Buff_Data[i * 32 + 4] * 256 + Buff_Data[i * 32 + 5]);
                        Robot_Analyzers.Robo_Compass = ((Buff_Data[i * 32 + 6] * 256 + Buff_Data[i * 32 + 7]) - 32768) / 10.0;

                        Robot_Analyzers.Robo_duty[0] = (Buff_Data[i * 32 + 8] * 256 + Buff_Data[i * 32 + 9]) - 255;
                        Robot_Analyzers.Robot_View_Delay = Buff_Data[i * 32 + 10];
                        Robot_Analyzers.Robo_duty[1] = (Buff_Data[i * 32 + 11] * 256 + Buff_Data[i * 32 + 12]) - 255;
                        Robot_Analyzers.Robo_Input_Val = Buff_Data[i * 32 + 13];
                        Robot_Analyzers.Robo_duty[2] = (Buff_Data[i * 32 + 14] * 256 + Buff_Data[i * 32 + 15]) - 255;
                        Robot_Analyzers.Robo_Input_Val2 = Buff_Data[i * 32 + 16];
                        Robot_Analyzers.Robo_duty[3] = (Buff_Data[i * 32 + 17] * 256 + Buff_Data[i * 32 + 18]) - 255;
                        Robot_Analyzers.Robo_Input_Val3 = Buff_Data[i * 32 + 19];

                        Robot_Analyzers.Robo_ADC1 = (uint)(Buff_Data[i * 32 + 20] * 256 + Buff_Data[i * 32 + 21]);
                        Robot_Analyzers.Robo_ADC2 = (uint)(Buff_Data[i * 32 + 22] * 256 + Buff_Data[i * 32 + 23]);
                        Robot_Analyzers.Robo_ADC3 = (uint)(Buff_Data[i * 32 + 24] * 256 + Buff_Data[i * 32 + 25]);

                        if (Robot_Analyzers.Robot_View_Delay < 10) Robot_Analyzers.Robot_View_Delay = 10;

                        Robot_Analyzers.Robo_A_Val = (uint)(Buff_Data[i * 32 + 26] * 256 + Buff_Data[i * 32 + 27]);
                        Robot_Analyzers.Robo_B_Val = (uint)(Buff_Data[i * 32 + 28] * 256 + Buff_Data[i * 32 + 29]);
                        Robot_Analyzers.Robo_C_Val = (uint)(Buff_Data[i * 32 + 30] * 256 + Buff_Data[i * 32 + 31]);
             
                        // hien thi du lieu
                        //_xyDataSeries_rba.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Compass);
                        Robot_Analyzers._xyDataSeries_rb2.Append(i, Robot_Analyzers.Robo_duty[0]);
                        Robot_Analyzers._xyDataSeries_rb3.Append(i, Robot_Analyzers.Robo_duty[1]);
                        Robot_Analyzers._xyDataSeries_rb4.Append(i, Robot_Analyzers.Robo_duty[2]);
                        Robot_Analyzers._xyDataSeries_rb5.Append(i, Robot_Analyzers.Robo_duty[3]);
                        Robot_Analyzers._xyDataSeries_rb6.Append(i, Robot_Analyzers.Robo_ADC1);
                        Robot_Analyzers._xyDataSeries_rb7.Append(i, Robot_Analyzers.Robo_ADC2);
                        Robot_Analyzers._xyDataSeries_rb8.Append(i, Robot_Analyzers.Robo_ADC3);
                        Robot_Analyzers._xyDataSeries_rb9.Append(i, Robot_Analyzers.Robo_Compass);
                        Robot_Analyzers._xyDataSeries_rb10.Append(i, Robot_Analyzers.Robo_A_Val);
                        Robot_Analyzers._xyDataSeries_rb11.Append(i, Robot_Analyzers.Robo_B_Val);
                        Robot_Analyzers._xyDataSeries_rb12.Append(i, Robot_Analyzers.Robo_C_Val);
                        Robot_Analyzers._xyDataSeries_rb13.Append(i, Robot_Analyzers.Robo_Encoder);

                        Robot_Analyzers._xyDataSeries_IO9.Append(i, Robot_Analyzers.Robo_Input_Val3);

                        if (Robot_Analyzers.Max_ADC <= Robot_Analyzers.Robo_ADC1) Robot_Analyzers.Max_ADC = Robot_Analyzers.Robo_ADC1 + 2000;
                        if (Robot_Analyzers.Max_ADC <= Robot_Analyzers.Robo_ADC2) Robot_Analyzers.Max_ADC = Robot_Analyzers.Robo_ADC2 + 2000;
                        if (Robot_Analyzers.Max_ADC <= Robot_Analyzers.Robo_ADC3) Robot_Analyzers.Max_ADC = Robot_Analyzers.Robo_ADC3 + 2000;

                        if (Robot_Analyzers.Max_Compass <= Robot_Analyzers.Robo_A_Val) Robot_Analyzers.Max_Compass = (int)(Robot_Analyzers.Robo_A_Val + 10);

                        if (Robot_Analyzers.Max_ABC <= Robot_Analyzers.Robo_B_Val) Robot_Analyzers.Max_ABC = Robot_Analyzers.Robo_B_Val + 100;
                        if (Robot_Analyzers.Max_ABC <= Robot_Analyzers.Robo_C_Val) Robot_Analyzers.Max_ABC = Robot_Analyzers.Robo_C_Val + 100;

                        if (Robot_Analyzers.Robo_Compass >= Robot_Analyzers.Max_Compass) Robot_Analyzers.Max_Compass = (Int32)(Robot_Analyzers.Robo_Compass + 10);
                        if (Robot_Analyzers.Robo_Compass <= Robot_Analyzers.Min_Compass) Robot_Analyzers.Min_Compass = (Int32)(Robot_Analyzers.Robo_Compass - 10);

                        if (Robot_Analyzers.Max_Compass >= 3000) Robot_Analyzers.Max_Compass = 3000;
                        if (Robot_Analyzers.Min_Compass <= -3000) Robot_Analyzers.Min_Compass = 3000;
                        if (Robot_Analyzers.Max_ABC >= 1000) Robot_Analyzers.Max_ABC = 1000;

                        if (Robot_Analyzers.Compass_input3_Max <= Robot_Analyzers.Robo_Input_Val3) Robot_Analyzers.Compass_input3_Max = Robot_Analyzers.Robo_Input_Val3 + 1;
                        //Compass_input3_Maxi
                        if (Robot_Analyzers.Compass_input3_Maxi < Robot_Analyzers.Compass_input3_Max)
                        {
                            Robot_Analyzers.Compass_input3_Maxi = Robot_Analyzers.Compass_input3_Max;
                            Y_SCope7.Dispatcher.Invoke(() => Y_SCope7.VisibleRange = new DoubleRange(0, Robot_Analyzers.Compass_input3_Max));
                        }

                        if (Robot_Analyzers.Max_ADCi < Robot_Analyzers.Max_ADC)
                        {
                            Robot_Analyzers.Max_ADCi = Robot_Analyzers.Max_ADC;
                            Y_SCope3.Dispatcher.Invoke(() => Y_SCope3.VisibleRange = new DoubleRange(0, Robot_Analyzers.Max_ADC));

                        }

                        if ((Robot_Analyzers.Max_Compassi < Robot_Analyzers.Max_Compass) || (Robot_Analyzers.Min_Compassi > Robot_Analyzers.Min_Compass))
                        {
                            Robot_Analyzers.Max_Compassi = Robot_Analyzers.Max_Compass;
                            Robot_Analyzers.Min_Compassi = Robot_Analyzers.Min_Compass;

                            Y_SCope5.Dispatcher.Invoke(() => Y_SCope5.VisibleRange = new DoubleRange(Robot_Analyzers.Min_Compass, Robot_Analyzers.Max_Compass));

                        }

                        if (Robot_Analyzers.Max_ABCi < Robot_Analyzers.Max_ABC)
                        {
                            Robot_Analyzers.Max_ABCi = Robot_Analyzers.Max_ABC;
                            Y_SCope6.Dispatcher.Invoke(() => Y_SCope6.VisibleRange = new DoubleRange(0, Robot_Analyzers.Max_ABC));

                        }            
                            
                    }

                    S1_Line1i.X1 = 30;
                    S2_Line1i.X1 = 30;
                    S3_Line1i.X1 = 30;
                    S4_Line1i.X1 = 30;
                    S5_Line1i.X1 = 30;

                    S1_Line1i_MouseMove(null, null);
                }
            }
        }


        private void Start_sAVE_LOG_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Robot_Analyzers.Robot_Packet_Save_File = 2;
        }

        private void Start_oPNEN_LOG_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            if (Robot_Analyzers.Robot_Thread_Run == 1)
            {
                Robot_Analyzers.Robot_Thread_Run = 0;
                Thread_ethenet2.Abort();
                Thread.Sleep(200);
            }
            else if (Robot_Analyzers.Robot_Thread_Run == 2)
            {
                Robot_Analyzers.Robot_Thread_Run = 0;
                Thread_ethenet4.Abort();
                Thread.Sleep(200);
            }

            Robot_Clear();
            Robot_Analyzers.Robot_Analyse_Start = 1;
            SCREEN_VIEWS2.IsChecked = false;
            Analyse_FEE_HAUI();
        }

        public void Reload_Thread()
        {
            if (Robot_Analyzers.Robot_View_Mode == 0)
            {
                Robot_Clear();

                Thread.Sleep(200);

                Thread_ethenet2 = new Thread(() => Robo_Signal());
                Thread_ethenet2.Start();
            }
            else if (Robot_Analyzers.Robot_View_Mode == 1)
            {
                Robot_Clear();

                Thread.Sleep(200);

                Thread_ethenet4 = new Thread(() => Robo_Signal2());
                Thread_ethenet4.Start();
            }
        }

        private void Elephant_Checked(object sender, RoutedEventArgs e)
        {
            if (Robot_Analyzers.Robot_Running == 1)
            {
                
                Robot_Analyzers.Robot_View_Mode = 0;
                Thread_ethenet4.Abort();

                Thread.Sleep(200);

                Robot_Clear();

                Thread.Sleep(200);

                Thread_ethenet2 = new Thread(() => Robo_Signal());
                Thread_ethenet2.Start();
            }
        }

        private void Rabit_Checked(object sender, RoutedEventArgs e)
        {
            if (Robot_Analyzers.Robot_Running == 1)
            {
                Robot_Analyzers.Robot_View_Mode = 1;
                Thread_ethenet2.Abort();

                Thread.Sleep(200);

                Robot_Clear();

                Thread.Sleep(200);

                Thread_ethenet4 = new Thread(() => Robo_Signal2());
                Thread_ethenet4.Start();
            }
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            //Thread_ethenet3.Abort();
            try
            {
                if (Robot_Analyzers.Robot_Thread_Run == 1) Thread_ethenet2.Abort();
            }
            catch { }
            //Thread_ethenet.Abort();
            //Thread_ethenet5.Abort();
            try 
            { 
                if (Robot_Analyzers.Robot_Thread_Run == 2) Thread_ethenet4.Abort(); 
            } catch { }

            Check_Counter.Stop();
            Check_USB.Stop();
            Thread_USB.Abort();    

            Rabit_Client.Close();
            Robo_Client.Close();
            Lidar_Client.Close();
            Compass_Client.Close();
        }
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            //Thread_ethenet3.Abort();
            try
            {
                if (Robot_Analyzers.Robot_Thread_Run == 1) Thread_ethenet2.Abort();
            }
            catch { }
            //Thread_ethenet.Abort();
            //Thread_ethenet5.Abort();
            try
            {
                if (Robot_Analyzers.Robot_Thread_Run == 2) Thread_ethenet4.Abort();
            }
            catch { }

            Check_Counter.Stop();
            Check_USB.Stop();
            Thread_USB.Abort();

            Rabit_Client.Close();
            Robo_Client.Close();
            Lidar_Client.Close();
            Compass_Client.Close();
        }

        #endregion

        #region Thread Ethenet
        public void Compass_Signal()
        {
            // kiểm tra kết nối PC1
            //UInt16 Lat0 = 0;
            UInt16 connect = 0;
            UInt16 Counters = 0;
            UInt16 Counters2 = 0;

            Sensor_Data_R1.Dispatcher.Invoke(() => Sensor_Data_R1.DataSeries = Robot_Analyzers._xyDataSeries_R1);
            Sensor_Data_R2.Dispatcher.Invoke(() => Sensor_Data_R2.DataSeries = Robot_Analyzers._xyDataSeries_R2);
            Sensor_Data_R3.Dispatcher.Invoke(() => Sensor_Data_R3.DataSeries = Robot_Analyzers._xyDataSeries_R3);
            Sensor_Data_R4.Dispatcher.Invoke(() => Sensor_Data_R4.DataSeries = Robot_Analyzers._xyDataSeries_R4);

            Sensor_Data_A1.Dispatcher.Invoke(() => Sensor_Data_A1.DataSeries = Robot_Analyzers._xyDataSeries_A1);
            Sensor_Data_A2.Dispatcher.Invoke(() => Sensor_Data_A2.DataSeries = Robot_Analyzers._xyDataSeries_A2);
            Sensor_Data_A3.Dispatcher.Invoke(() => Sensor_Data_A3.DataSeries = Robot_Analyzers._xyDataSeries_A3);
            Sensor_Data_A4.Dispatcher.Invoke(() => Sensor_Data_A4.DataSeries = Robot_Analyzers._xyDataSeries_A4);
            Sensor_Data_A5.Dispatcher.Invoke(() => Sensor_Data_A5.DataSeries = Robot_Analyzers._xyDataSeries_A5);

            while (true)
            {
                try
                {
                    Robot_Analyzers.Compass_Frame = Compass_Client.Receive(ref Compass_Clienti);

                    Robot_Analyzers.Compass_R1 = (Int16)(Robot_Analyzers.Compass_Frame[3] * 256 + Robot_Analyzers.Compass_Frame[4]);
                    Robot_Analyzers.Compass_R2 = (Int16)(Robot_Analyzers.Compass_Frame[5] * 256 + Robot_Analyzers.Compass_Frame[6]);
                    Robot_Analyzers.Compass_R3 = (Int16)(Robot_Analyzers.Compass_Frame[7] * 256 + Robot_Analyzers.Compass_Frame[8]);
                    Robot_Analyzers.Compass_R4 = (Int16)(Robot_Analyzers.Compass_Frame[9] * 256 + Robot_Analyzers.Compass_Frame[10]);

                    Robot_Analyzers.Compass_A1 = ((Int16)(Robot_Analyzers.Compass_Frame[11] * 256 + Robot_Analyzers.Compass_Frame[12])) / 10.0;
                    Robot_Analyzers.Compass_A2 = ((Int16)(Robot_Analyzers.Compass_Frame[13] * 256 + Robot_Analyzers.Compass_Frame[14])) / 10.0;
                    Robot_Analyzers.Compass_A3 = ((Int16)(Robot_Analyzers.Compass_Frame[15] * 256 + Robot_Analyzers.Compass_Frame[16])) / 10.0;
                    Robot_Analyzers.Compass_A4 = ((Int16)(Robot_Analyzers.Compass_Frame[17] * 256 + Robot_Analyzers.Compass_Frame[18])) / 10.0;

                    Robot_Analyzers.Compass_A5 = Robot_Analyzers.Robo_Compass;

                    Robot_Analyzers.Compass_Offset1 = (Int16)(Robot_Analyzers.Compass_Frame[19] * 256 + Robot_Analyzers.Compass_Frame[20]);
                    Robot_Analyzers.Compass_Offset2 = (Int16)(Robot_Analyzers.Compass_Frame[21] * 256 + Robot_Analyzers.Compass_Frame[22]);
                    Robot_Analyzers.Compass_Offset3 = (Int16)(Robot_Analyzers.Compass_Frame[23] * 256 + Robot_Analyzers.Compass_Frame[24]);
                    Robot_Analyzers.Compass_Offset4 = (Int16)(Robot_Analyzers.Compass_Frame[25] * 256 + Robot_Analyzers.Compass_Frame[26]);

                    if (Robot_Analyzers.Robot_View_SYNC == 1)
                    {
                        Robot_Analyzers.Robo_Restart2 = Robot_Analyzers.Robo_Restart;
                    }


                    //if (Robo_Restart == 0) Robo_Sample = 0; //else Robo_Sample++;
                    //_re_R1[Robo_Encoder] = Robo_ADC1;
                    //_im_R1[Robo_Encoder] = Robo_Encoder;

                    if (Robot_Analyzers.Robot_View_SYNC == 0) // xu ly doc lap
                    {
                        if (Robot_Analyzers.Robot_View_Stop2 == 0)
                        {
                            Robot_Analyzers.Compass_Sample++;

                            RAM_D1.Dispatcher.Invoke(() => RAM_D1.Text = string.Concat(Robot_Analyzers.Compass_R1));
                            RAM_D2.Dispatcher.Invoke(() => RAM_D2.Text = string.Concat(Robot_Analyzers.Compass_R2));
                            RAM_D3.Dispatcher.Invoke(() => RAM_D3.Text = string.Concat(Robot_Analyzers.Compass_R3));
                            RAM_D4.Dispatcher.Invoke(() => RAM_D4.Text = string.Concat(Robot_Analyzers.Compass_R4));

                            Result_D1.Dispatcher.Invoke(() => Result_D1.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_A1, 2)));
                            Result_D2.Dispatcher.Invoke(() => Result_D2.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_A2, 2)));
                            Result_D3.Dispatcher.Invoke(() => Result_D3.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_A3, 2)));
                            Result_D4.Dispatcher.Invoke(() => Result_D4.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_A4, 2)));
                            Result_D5.Dispatcher.Invoke(() => Result_D5.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_A5, 2)));

                            RAM_OF1.Dispatcher.Invoke(() => RAM_OF1.Text = string.Concat(Robot_Analyzers.Compass_Offset1));
                            RAM_OF2.Dispatcher.Invoke(() => RAM_OF2.Text = string.Concat(Robot_Analyzers.Compass_Offset2));
                            RAM_OF3.Dispatcher.Invoke(() => RAM_OF3.Text = string.Concat(Robot_Analyzers.Compass_Offset3));
                            RAM_OF4.Dispatcher.Invoke(() => RAM_OF4.Text = string.Concat(Robot_Analyzers.Compass_Offset4));

                            if (Robot_Analyzers.Compass_Sample > 20000) Robot_Analyzers.Clear_Request2 = 1;

                            if ((Robot_Analyzers.Compass_Sample >= 100) && (Robot_Analyzers.Compass_Sample % 100 == 0))
                            {
                                X_SCope1.Dispatcher.Invoke(() => X_SCope1.VisibleRange = new DoubleRange(0, Robot_Analyzers.Compass_Sample + 100));
                                X_SCope2.Dispatcher.Invoke(() => X_SCope2.VisibleRange = new DoubleRange(0, Robot_Analyzers.Compass_Sample + 100));
                            }

                            Robot_Analyzers.Compass_A1 = 0;

                            if (Robot_Analyzers.Compass_Raw_Max <= Robot_Analyzers.Compass_R1) Robot_Analyzers.Compass_Raw_Max = Robot_Analyzers.Compass_R1 + 50;
                            if (Robot_Analyzers.Compass_Raw_Max <= Robot_Analyzers.Compass_R2) Robot_Analyzers.Compass_Raw_Max = Robot_Analyzers.Compass_R2 + 50;
                            if (Robot_Analyzers.Compass_Raw_Max <= Robot_Analyzers.Compass_R3) Robot_Analyzers.Compass_Raw_Max = Robot_Analyzers.Compass_R3 + 50;
                            if (Robot_Analyzers.Compass_Raw_Max <= Robot_Analyzers.Compass_R4) Robot_Analyzers.Compass_Raw_Max = Robot_Analyzers.Compass_R4 + 50;


                            if (Robot_Analyzers.Compass_Raw_Min >= Robot_Analyzers.Compass_R1) Robot_Analyzers.Compass_Raw_Min = Robot_Analyzers.Compass_R1 - 50;
                            if (Robot_Analyzers.Compass_Raw_Min >= Robot_Analyzers.Compass_R2) Robot_Analyzers.Compass_Raw_Min = Robot_Analyzers.Compass_R2 - 50;
                            if (Robot_Analyzers.Compass_Raw_Min >= Robot_Analyzers.Compass_R3) Robot_Analyzers.Compass_Raw_Min = Robot_Analyzers.Compass_R3 - 50;
                            if (Robot_Analyzers.Compass_Raw_Min >= Robot_Analyzers.Compass_R4) Robot_Analyzers.Compass_Raw_Min = Robot_Analyzers.Compass_R4 - 50;

                            Robot_Analyzers.Compass_A1 = 0;


                            if (Robot_Analyzers.Compass_Result_Max <= Robot_Analyzers.Compass_A1) Robot_Analyzers.Compass_Result_Max = Robot_Analyzers.Compass_A1 + 1.0;
                            if (Robot_Analyzers.Compass_Result_Max <= Robot_Analyzers.Compass_A2) Robot_Analyzers.Compass_Result_Max = Robot_Analyzers.Compass_A2 + 1.0;
                            if (Robot_Analyzers.Compass_Result_Max <= Robot_Analyzers.Compass_A3) Robot_Analyzers.Compass_Result_Max = Robot_Analyzers.Compass_A3 + 1.0;
                            if (Robot_Analyzers.Compass_Result_Max <= Robot_Analyzers.Compass_A4) Robot_Analyzers.Compass_Result_Max = Robot_Analyzers.Compass_A4 + 1.0;
                            if (Robot_Analyzers.Compass_Result_Max <= Robot_Analyzers.Compass_A5) Robot_Analyzers.Compass_Result_Max = Robot_Analyzers.Compass_A5 + 1.0;


                            if (Robot_Analyzers.Compass_Result_Min >= Robot_Analyzers.Compass_A1) Robot_Analyzers.Compass_Result_Min = Robot_Analyzers.Compass_A1 - 1.0;
                            if (Robot_Analyzers.Compass_Result_Min >= Robot_Analyzers.Compass_A2) Robot_Analyzers.Compass_Result_Min = Robot_Analyzers.Compass_A2 - 1.0;
                            if (Robot_Analyzers.Compass_Result_Min >= Robot_Analyzers.Compass_A3) Robot_Analyzers.Compass_Result_Min = Robot_Analyzers.Compass_A3 - 1.0;
                            if (Robot_Analyzers.Compass_Result_Min >= Robot_Analyzers.Compass_A4) Robot_Analyzers.Compass_Result_Min = Robot_Analyzers.Compass_A4 - 1.0;
                            if (Robot_Analyzers.Compass_Result_Min >= Robot_Analyzers.Compass_A5) Robot_Analyzers.Compass_Result_Min = Robot_Analyzers.Compass_A5 - 1.0;

                            try
                            {
                                //_xyDataSeries_rb1.Clear();
                                //_xyDataSeries_rb1.Append(_re_R1, _im_R1);
                                Robot_Analyzers._xyDataSeries_R1.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_R1);
                                Robot_Analyzers._xyDataSeries_R2.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_R2);
                                Robot_Analyzers._xyDataSeries_R3.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_R3);
                                Robot_Analyzers._xyDataSeries_R4.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_R4);

                                Robot_Analyzers._xyDataSeries_A1.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_A1);
                                Robot_Analyzers._xyDataSeries_A2.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_A2);
                                Robot_Analyzers._xyDataSeries_A3.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_A3);
                                Robot_Analyzers._xyDataSeries_A4.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_A4);
                                Robot_Analyzers._xyDataSeries_A5.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_A5);
                            }
                            catch
                            {

                            }

                            if (Robot_Analyzers.Compass_Raw_Maxi < Robot_Analyzers.Compass_Raw_Max)
                            {
                                Robot_Analyzers.Compass_Raw_Maxi = Robot_Analyzers.Compass_Raw_Max;
                                Y_SCope1.Dispatcher.Invoke(() => Y_SCope1.VisibleRange = new DoubleRange(Robot_Analyzers.Compass_Raw_Min, Robot_Analyzers.Compass_Raw_Max));
                            }

                            if (Robot_Analyzers.Compass_Raw_Mini > Robot_Analyzers.Compass_Raw_Min)
                            {
                                Robot_Analyzers.Compass_Raw_Mini = Robot_Analyzers.Compass_Raw_Min;
                                Y_SCope1.Dispatcher.Invoke(() => Y_SCope1.VisibleRange = new DoubleRange(Robot_Analyzers.Compass_Raw_Min, Robot_Analyzers.Compass_Raw_Max));
                            }

                            if (Robot_Analyzers.Compass_Result_Maxi < Robot_Analyzers.Compass_Result_Max)
                            {
                                Robot_Analyzers.Compass_Result_Maxi = Robot_Analyzers.Compass_Result_Max;
                                Y_SCope2.Dispatcher.Invoke(() => Y_SCope2.VisibleRange = new DoubleRange(Robot_Analyzers.Compass_Result_Min, Robot_Analyzers.Compass_Result_Max));
                            }

                            if (Robot_Analyzers.Compass_Result_Mini > Robot_Analyzers.Compass_Result_Min)
                            {
                                Robot_Analyzers.Compass_Result_Mini = Robot_Analyzers.Compass_Result_Min;
                                Y_SCope2.Dispatcher.Invoke(() => Y_SCope2.VisibleRange = new DoubleRange(Robot_Analyzers.Compass_Result_Min, Robot_Analyzers.Compass_Result_Max));
                            }
                        }
                    }
                    else if (Robot_Analyzers.Robot_View_SYNC == 1)
                    {
                        Robot_Analyzers.Robot_View_Clear2 = Robot_Analyzers.Robo_Restart2;

                        if ((Robot_Analyzers.Robot_View_Cleari2 != Robot_Analyzers.Robot_View_Clear2) && (Robot_Analyzers.Robot_View_Clear2 < 20))
                        {
                            Robot_Analyzers.Robot_View_Cleari2 = Robot_Analyzers.Robot_View_Clear2;
                            Robot_Analyzers.Clear_Request2 = 1;
                        }
                            
                        
                        if ((Robot_Analyzers.Robo_Restart2 >= 1) && (Robot_Analyzers.Robo_Restart2 < 20))
                        {
                            Robot_Analyzers.Compass_Sample++;


                            RAM_D1.Dispatcher.Invoke(() => RAM_D1.Text = string.Concat(Robot_Analyzers.Compass_R1));
                            RAM_D2.Dispatcher.Invoke(() => RAM_D2.Text = string.Concat(Robot_Analyzers.Compass_R2));
                            RAM_D3.Dispatcher.Invoke(() => RAM_D3.Text = string.Concat(Robot_Analyzers.Compass_R3));
                            RAM_D4.Dispatcher.Invoke(() => RAM_D4.Text = string.Concat(Robot_Analyzers.Compass_R4));

                            Result_D1.Dispatcher.Invoke(() => Result_D1.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_A1, 2)));
                            Result_D2.Dispatcher.Invoke(() => Result_D2.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_A2, 2)));
                            Result_D3.Dispatcher.Invoke(() => Result_D3.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_A3, 2)));
                            Result_D4.Dispatcher.Invoke(() => Result_D4.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_A4, 2)));
                            Result_D5.Dispatcher.Invoke(() => Result_D5.Text = string.Concat(Math.Round(Robot_Analyzers.Compass_A5, 2)));

                            RAM_OF1.Dispatcher.Invoke(() => RAM_OF1.Text = string.Concat(Robot_Analyzers.Compass_Offset1));
                            RAM_OF2.Dispatcher.Invoke(() => RAM_OF2.Text = string.Concat(Robot_Analyzers.Compass_Offset2));
                            RAM_OF3.Dispatcher.Invoke(() => RAM_OF3.Text = string.Concat(Robot_Analyzers.Compass_Offset3));
                            RAM_OF4.Dispatcher.Invoke(() => RAM_OF4.Text = string.Concat(Robot_Analyzers.Compass_Offset4));

                            if (Robot_Analyzers.Compass_Sample > 20000) Robot_Analyzers.Clear_Request2 = 1;

                            if ((Robot_Analyzers.Compass_Sample >= 100) && (Robot_Analyzers.Compass_Sample % 100 == 0))
                            {
                                X_SCope1.Dispatcher.Invoke(() => X_SCope1.VisibleRange = new DoubleRange(0, Robot_Analyzers.Compass_Sample + 100));
                                X_SCope2.Dispatcher.Invoke(() => X_SCope2.VisibleRange = new DoubleRange(0, Robot_Analyzers.Compass_Sample + 100));
                            }

                            Robot_Analyzers.Compass_A1 = 0;

                            if (Robot_Analyzers.Compass_Raw_Max <= Robot_Analyzers.Compass_R1) Robot_Analyzers.Compass_Raw_Max = Robot_Analyzers.Compass_R1 + 50;
                            if (Robot_Analyzers.Compass_Raw_Max <= Robot_Analyzers.Compass_R2) Robot_Analyzers.Compass_Raw_Max = Robot_Analyzers.Compass_R2 + 50;
                            if (Robot_Analyzers.Compass_Raw_Max <= Robot_Analyzers.Compass_R3) Robot_Analyzers.Compass_Raw_Max = Robot_Analyzers.Compass_R3 + 50;
                            if (Robot_Analyzers.Compass_Raw_Max <= Robot_Analyzers.Compass_R4) Robot_Analyzers.Compass_Raw_Max = Robot_Analyzers.Compass_R4 + 50;


                            if (Robot_Analyzers.Compass_Raw_Min >= Robot_Analyzers.Compass_R1) Robot_Analyzers.Compass_Raw_Min = Robot_Analyzers.Compass_R1 - 50;
                            if (Robot_Analyzers.Compass_Raw_Min >= Robot_Analyzers.Compass_R2) Robot_Analyzers.Compass_Raw_Min = Robot_Analyzers.Compass_R2 - 50;
                            if (Robot_Analyzers.Compass_Raw_Min >= Robot_Analyzers.Compass_R3) Robot_Analyzers.Compass_Raw_Min = Robot_Analyzers.Compass_R3 - 50;
                            if (Robot_Analyzers.Compass_Raw_Min >= Robot_Analyzers.Compass_R4) Robot_Analyzers.Compass_Raw_Min = Robot_Analyzers.Compass_R4 - 50;


                            if (Robot_Analyzers.Compass_Result_Max <= Robot_Analyzers.Compass_A1) Robot_Analyzers.Compass_Result_Max = Robot_Analyzers.Compass_A1 + 1.0;
                            if (Robot_Analyzers.Compass_Result_Max <= Robot_Analyzers.Compass_A2) Robot_Analyzers.Compass_Result_Max = Robot_Analyzers.Compass_A2 + 1.0;
                            if (Robot_Analyzers.Compass_Result_Max <= Robot_Analyzers.Compass_A3) Robot_Analyzers.Compass_Result_Max = Robot_Analyzers.Compass_A3 + 1.0;
                            if (Robot_Analyzers.Compass_Result_Max <= Robot_Analyzers.Compass_A4) Robot_Analyzers.Compass_Result_Max = Robot_Analyzers.Compass_A4 + 1.0;
                            if (Robot_Analyzers.Compass_Result_Max <= Robot_Analyzers.Compass_A5) Robot_Analyzers.Compass_Result_Max = Robot_Analyzers.Compass_A5 + 1.0;


                            if (Robot_Analyzers.Compass_Result_Min >= Robot_Analyzers.Compass_A1) Robot_Analyzers.Compass_Result_Min = Robot_Analyzers.Compass_A1 - 1.0;
                            if (Robot_Analyzers.Compass_Result_Min >= Robot_Analyzers.Compass_A2) Robot_Analyzers.Compass_Result_Min = Robot_Analyzers.Compass_A2 - 1.0;
                            if (Robot_Analyzers.Compass_Result_Min >= Robot_Analyzers.Compass_A3) Robot_Analyzers.Compass_Result_Min = Robot_Analyzers.Compass_A3 - 1.0;
                            if (Robot_Analyzers.Compass_Result_Min >= Robot_Analyzers.Compass_A4) Robot_Analyzers.Compass_Result_Min = Robot_Analyzers.Compass_A4 - 1.0;
                            if (Robot_Analyzers.Compass_Result_Min >= Robot_Analyzers.Compass_A5) Robot_Analyzers.Compass_Result_Min = Robot_Analyzers.Compass_A5 - 1.0;

                            try
                            {
                                //_xyDataSeries_rb1.Clear();
                                //_xyDataSeries_rb1.Append(_re_R1, _im_R1);
                                Robot_Analyzers._xyDataSeries_R1.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_R1);
                                Robot_Analyzers._xyDataSeries_R2.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_R2);
                                Robot_Analyzers._xyDataSeries_R3.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_R3);
                                Robot_Analyzers._xyDataSeries_R4.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_R4);

                                Robot_Analyzers._xyDataSeries_A1.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_A1);
                                Robot_Analyzers._xyDataSeries_A2.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_A2);
                                Robot_Analyzers._xyDataSeries_A3.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_A3);
                                Robot_Analyzers._xyDataSeries_A4.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_A4);
                                Robot_Analyzers._xyDataSeries_A5.Append(Robot_Analyzers.Compass_Sample, Robot_Analyzers.Compass_A5);
                            }
                            catch
                            {

                            }

                            if (Robot_Analyzers.Compass_Raw_Maxi < Robot_Analyzers.Compass_Raw_Max)
                            {
                                Robot_Analyzers.Compass_Raw_Maxi = Robot_Analyzers.Compass_Raw_Max;
                                Y_SCope1.Dispatcher.Invoke(() => Y_SCope1.VisibleRange = new DoubleRange(Robot_Analyzers.Compass_Raw_Min, Robot_Analyzers.Compass_Raw_Max));
                            }

                            if (Robot_Analyzers.Compass_Raw_Mini > Robot_Analyzers.Compass_Raw_Min)
                            {
                                Robot_Analyzers.Compass_Raw_Mini = Robot_Analyzers.Compass_Raw_Min;
                                Y_SCope1.Dispatcher.Invoke(() => Y_SCope1.VisibleRange = new DoubleRange(Robot_Analyzers.Compass_Raw_Min, Robot_Analyzers.Compass_Raw_Max));
                            }

                            if (Robot_Analyzers.Compass_Result_Maxi < Robot_Analyzers.Compass_Result_Max)
                            {
                                Robot_Analyzers.Compass_Result_Maxi = Robot_Analyzers.Compass_Result_Max;
                                Y_SCope2.Dispatcher.Invoke(() => Y_SCope2.VisibleRange = new DoubleRange(Robot_Analyzers.Compass_Result_Min, Robot_Analyzers.Compass_Result_Max));
                            }

                            if (Robot_Analyzers.Compass_Result_Mini > Robot_Analyzers.Compass_Result_Min)
                            {
                                Robot_Analyzers.Compass_Result_Mini = Robot_Analyzers.Compass_Result_Min;
                                Y_SCope2.Dispatcher.Invoke(() => Y_SCope2.VisibleRange = new DoubleRange(Robot_Analyzers.Compass_Result_Min, Robot_Analyzers.Compass_Result_Max));
                            }
                        }
                    }

                    if (connect == 0)
                    {
                        connect = 1;
                        Connect3.Dispatcher.Invoke(() => Connect3.Fill = Brushes.Red);
                    }
                    else if (connect == 1)
                    {
                        connect = 0;
                        Connect3.Dispatcher.Invoke(() => Connect3.Fill = Brushes.Green);
                    }


                    if (Robot_Analyzers.Clear_Request2 == 1)
                    {
                        Robot_Analyzers.Clear_Request2 = 0;
                        Robot_Analyzers.Compass_Sample = 0;

                        S8_Line1i.Dispatcher.Invoke(() => S8_Line1i.X1 = 30);
                        S9_Line1i.Dispatcher.Invoke(() => S9_Line1i.X1 = 30);

                        X_SCope1.Dispatcher.Invoke(() => X_SCope1.VisibleRange = new DoubleRange(0, 100));
                        X_SCope2.Dispatcher.Invoke(() => X_SCope2.VisibleRange = new DoubleRange(0, 100));

                        Robot_Analyzers.Compass_Raw_Max = 10;
                        Robot_Analyzers.Compass_Raw_Min = -10;

                        Robot_Analyzers.Compass_Result_Max = 10;
                        Robot_Analyzers.Compass_Result_Min = -10;

                        Robot_Analyzers.Compass_Raw_Maxi = 10;
                        Robot_Analyzers.Compass_Raw_Mini = -10;

                        Robot_Analyzers.Compass_Result_Maxi = 10;
                        Robot_Analyzers.Compass_Result_Mini = -10;

                        Y_SCope1.Dispatcher.Invoke(() => Y_SCope1.VisibleRange = new DoubleRange(Robot_Analyzers.Compass_Raw_Min, Robot_Analyzers.Compass_Raw_Max));
                        Y_SCope2.Dispatcher.Invoke(() => Y_SCope2.VisibleRange = new DoubleRange(Robot_Analyzers.Compass_Result_Min, Robot_Analyzers.Compass_Result_Max));

                        Robot_Analyzers._xyDataSeries_R1 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_R2 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_R3 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_R4 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_A1 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_A2 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_A3 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_A4 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_A5 = new XyDataSeries<double>();

                        Sensor_Data_R1.Dispatcher.Invoke(() => Sensor_Data_R1.DataSeries = Robot_Analyzers._xyDataSeries_R1);
                        Sensor_Data_R2.Dispatcher.Invoke(() => Sensor_Data_R2.DataSeries = Robot_Analyzers._xyDataSeries_R2);
                        Sensor_Data_R3.Dispatcher.Invoke(() => Sensor_Data_R3.DataSeries = Robot_Analyzers._xyDataSeries_R3);
                        Sensor_Data_R4.Dispatcher.Invoke(() => Sensor_Data_R4.DataSeries = Robot_Analyzers._xyDataSeries_R4);

                        Sensor_Data_A1.Dispatcher.Invoke(() => Sensor_Data_A1.DataSeries = Robot_Analyzers._xyDataSeries_A1);
                        Sensor_Data_A2.Dispatcher.Invoke(() => Sensor_Data_A2.DataSeries = Robot_Analyzers._xyDataSeries_A2);
                        Sensor_Data_A3.Dispatcher.Invoke(() => Sensor_Data_A3.DataSeries = Robot_Analyzers._xyDataSeries_A3);
                        Sensor_Data_A4.Dispatcher.Invoke(() => Sensor_Data_A4.DataSeries = Robot_Analyzers._xyDataSeries_A4);
                        Sensor_Data_A5.Dispatcher.Invoke(() => Sensor_Data_A5.DataSeries = Robot_Analyzers._xyDataSeries_A5);


                        Robot_Analyzers.Compass_Sample = 0;

                    }


                }
                catch
                {

                }
            }
        }

        public void Robo_Signal()
        {
            Robot_Analyzers.Robot_Thread_Run = 1;

            Robot_Analyzers.connect = 0;
            Robot_Analyzers.Counters = 0;
            Robot_Analyzers.Counters2 = 0;

            Robot_Analyzers.Max_ADC = 0;
            Robot_Analyzers.Max_Compass = 0;
            Robot_Analyzers.Min_Compass = 0;
            Robot_Analyzers.Max_ABC = 100;

            Robot_Analyzers.Max_ADCi = 0;
            Robot_Analyzers.Max_Compassi = 0;
            Robot_Analyzers.Min_Compassi = 0;
            Robot_Analyzers.Max_ABCi = 100;

            //Sensor_Compass.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = _xyDataSeries_rba);
            Sensor_Duty1.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = Robot_Analyzers._xyDataSeries_rb2);
            Sensor_Duty2.Dispatcher.Invoke(() => Sensor_Duty2.DataSeries = Robot_Analyzers._xyDataSeries_rb3);
            Sensor_Duty3.Dispatcher.Invoke(() => Sensor_Duty3.DataSeries = Robot_Analyzers._xyDataSeries_rb4);
            Sensor_Duty4.Dispatcher.Invoke(() => Sensor_Duty4.DataSeries = Robot_Analyzers._xyDataSeries_rb5);

            Sensor_adcData1.Dispatcher.Invoke(() => Sensor_adcData1.DataSeries = Robot_Analyzers._xyDataSeries_rb6);
            Sensor_adcData2.Dispatcher.Invoke(() => Sensor_adcData2.DataSeries = Robot_Analyzers._xyDataSeries_rb7);
            Sensor_adcData3.Dispatcher.Invoke(() => Sensor_adcData3.DataSeries = Robot_Analyzers._xyDataSeries_rb8);
            Sensor_Encoder.Dispatcher.Invoke(() => Sensor_Encoder.DataSeries = Robot_Analyzers._xyDataSeries_rb13);

            Sensor_compassData.Dispatcher.Invoke(() => Sensor_compassData.DataSeries = Robot_Analyzers._xyDataSeries_rb9);

            Sensor_A_Data.Dispatcher.Invoke(() => Sensor_A_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb10);
            Sensor_B_Data.Dispatcher.Invoke(() => Sensor_B_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb11);
            Sensor_C_Data.Dispatcher.Invoke(() => Sensor_C_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb12);

            Sensor_lOGIC9.Dispatcher.Invoke(() => Sensor_lOGIC9.DataSeries = Robot_Analyzers._xyDataSeries_IO9);

            while (true)
            {
                try
                {
                    Robot_Analyzers.Robo_Frame = Robo_Client.Receive(ref Robo_Clienti);
                    //if (Robot_Analyzers.Robot_View_Mode == 0) Robot_Analyzers.Robot_Analyse = 1;

                    Robot_Analyzers.Robo_Restart = Robot_Analyzers.Robo_Frame[3];
                    Robot_Analyzers.Robo_Encoder = (Robot_Analyzers.Robo_Frame[4] * 256 + Robot_Analyzers.Robo_Frame[5]);
                    Robot_Analyzers.Robo_Compass = ((Robot_Analyzers.Robo_Frame[6] * 256 + Robot_Analyzers.Robo_Frame[7]) - 32768) / 10.0;

                    Robot_Analyzers.Robo_duty[0] = (Robot_Analyzers.Robo_Frame[8] * 256 + Robot_Analyzers.Robo_Frame[9]) - 255;
                    Robot_Analyzers.Robot_View_Delay = Robot_Analyzers.Robo_Frame[10];
                    Robot_Analyzers.Robo_duty[1] = (Robot_Analyzers.Robo_Frame[11] * 256 + Robot_Analyzers.Robo_Frame[12]) - 255;
                    Robot_Analyzers.Robo_Input_Val = Robot_Analyzers.Robo_Frame[13];
                    Robot_Analyzers.Robo_duty[2] = (Robot_Analyzers.Robo_Frame[14] * 256 + Robot_Analyzers.Robo_Frame[15]) - 255;
                    Robot_Analyzers.Robo_Input_Val2 = Robot_Analyzers.Robo_Frame[16];
                    Robot_Analyzers.Robo_duty[3] = (Robot_Analyzers.Robo_Frame[17] * 256 + Robot_Analyzers.Robo_Frame[18]) - 255;
                    Robot_Analyzers.Robo_Input_Val3 = Robot_Analyzers.Robo_Frame[19];

                    Robot_Analyzers.Robo_ADC1 = (uint)(Robot_Analyzers.Robo_Frame[20] * 256 + Robot_Analyzers.Robo_Frame[21]);
                    Robot_Analyzers.Robo_ADC2 = (uint)(Robot_Analyzers.Robo_Frame[22] * 256 + Robot_Analyzers.Robo_Frame[23]);
                    Robot_Analyzers.Robo_ADC3 = (uint)(Robot_Analyzers.Robo_Frame[24] * 256 + Robot_Analyzers.Robo_Frame[25]);

                    if (Robot_Analyzers.Robot_View_Delay < 10) Robot_Analyzers.Robot_View_Delay = 10;

                    Robot_Analyzers.Robo_A_Val = (uint)(Robot_Analyzers.Robo_Frame[26] * 256 + Robot_Analyzers.Robo_Frame[27]);
                    Robot_Analyzers.Robo_B_Val = (uint)(Robot_Analyzers.Robo_Frame[28] * 256 + Robot_Analyzers.Robo_Frame[29]);
                    Robot_Analyzers.Robo_C_Val = (uint)(Robot_Analyzers.Robo_Frame[30] * 256 + Robot_Analyzers.Robo_Frame[31]);


                    Robot_Analyzers.Robot_View_Clear = Robot_Analyzers.Robo_Restart;
                    if (Robot_Analyzers.Robo_Restart == 0) 
                    {
                        Robot_Analyzers.Robot_Packet_Save_File = 0;
                    }                         
                    else if ((Robot_Analyzers.Robo_Restart >= 1) && (Robot_Analyzers.Robo_Restart < 20))
                    {
                        Robot_Analyzers.Robot_Packet_Save_File = 1;
                    }
                    else if (Robot_Analyzers.Robo_Restart == 21)
                    {
                        Robot_Analyzers.Robot_Packet_Save_File = 2;
                    }

                    Save_Packet1();


                    if ((Robot_Analyzers.Robot_View_Cleari != Robot_Analyzers.Robot_View_Clear) && (Robot_Analyzers.Robot_View_Clear < 20))
                    {
                        Robot_Analyzers.Robot_View_Cleari = Robot_Analyzers.Robot_View_Clear;
                        Robot_Analyzers.Clear_Request1 = 1;
                    }

                    if (Robot_Analyzers.Robot_View_Stop == 0)
                    {
                        //if (Robot_Analyzers.Robot_View_Mode == 0)
                        //{
                        if ((Robot_Analyzers.Robo_Restart >= 1) && (Robot_Analyzers.Robo_Restart < 20))
                        {
                            Robot_Analyzers.Robo_Sample++;

                            if ((Robot_Analyzers.Robo_Sample % 5) == 0)
                            {
                                RB_tbx.Dispatcher.Invoke(() => RB_tbx.Text = string.Concat(Robot_Analyzers.Robo_Sample));
                                RB_tbx_time.Dispatcher.Invoke(() => RB_tbx_time.Text = string.Concat(Math.Round((Robot_Analyzers.Robo_Sample * Robot_Analyzers.Robot_View_Delay) / 1000.0, 2), "s"));
                            }

                            if (Robot_Analyzers.Robo_Sample > 20000) Robot_Analyzers.Clear_Request1 = 1;

                            if ((Robot_Analyzers.Robo_Sample >= 150) && (Robot_Analyzers.Robo_Sample % 150 == 0))
                            {
                                X_SCope4.Dispatcher.Invoke(() => X_SCope4.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                X_SCope3.Dispatcher.Invoke(() => X_SCope3.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                X_SCope5.Dispatcher.Invoke(() => X_SCope5.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                X_SCope6.Dispatcher.Invoke(() => X_SCope6.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                X_SCope7.Dispatcher.Invoke(() => X_SCope7.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                            }
                        }


                        ////try
                        ////{
                            //if (Robot_Analyzers.Robot_View_Mode == 0)
                            //{
                        if ((Robot_Analyzers.Robo_Restart >= 1) && (Robot_Analyzers.Robo_Restart < 20))
                        {
                            //_xyDataSeries_rba.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Compass);
                            Robot_Analyzers._xyDataSeries_rb2.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[0]);
                            Robot_Analyzers._xyDataSeries_rb3.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[1]);
                            Robot_Analyzers._xyDataSeries_rb4.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[2]);
                            Robot_Analyzers._xyDataSeries_rb5.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[3]);
                            Robot_Analyzers._xyDataSeries_rb6.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_ADC1);
                            Robot_Analyzers._xyDataSeries_rb7.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_ADC2);
                            Robot_Analyzers._xyDataSeries_rb8.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_ADC3);
                            Robot_Analyzers._xyDataSeries_rb9.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Compass);
                            Robot_Analyzers._xyDataSeries_rb10.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_A_Val);
                            Robot_Analyzers._xyDataSeries_rb11.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_B_Val);
                            Robot_Analyzers._xyDataSeries_rb12.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_C_Val);
                            Robot_Analyzers._xyDataSeries_rb13.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Encoder);

                            Robot_Analyzers._xyDataSeries_IO9.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Input_Val3);

                            if (Robot_Analyzers.Max_ADC <= Robot_Analyzers.Robo_ADC1) Robot_Analyzers.Max_ADC = Robot_Analyzers.Robo_ADC1 + 2000;
                            if (Robot_Analyzers.Max_ADC <= Robot_Analyzers.Robo_ADC2) Robot_Analyzers.Max_ADC = Robot_Analyzers.Robo_ADC2 + 2000;
                            if (Robot_Analyzers.Max_ADC <= Robot_Analyzers.Robo_ADC3) Robot_Analyzers.Max_ADC = Robot_Analyzers.Robo_ADC3 + 2000;

                            if (Robot_Analyzers.Max_Compass <= Robot_Analyzers.Robo_A_Val) Robot_Analyzers.Max_Compass = (int)(Robot_Analyzers.Robo_A_Val + 10);

                            if (Robot_Analyzers.Max_ABC <= Robot_Analyzers.Robo_B_Val) Robot_Analyzers.Max_ABC = Robot_Analyzers.Robo_B_Val + 100;
                            if (Robot_Analyzers.Max_ABC <= Robot_Analyzers.Robo_C_Val) Robot_Analyzers.Max_ABC = Robot_Analyzers.Robo_C_Val + 100;

                            if (Robot_Analyzers.Robo_Compass >= Robot_Analyzers.Max_Compass) Robot_Analyzers.Max_Compass = (Int32)(Robot_Analyzers.Robo_Compass + 10);
                            if (Robot_Analyzers.Robo_Compass <= Robot_Analyzers.Min_Compass) Robot_Analyzers.Min_Compass = (Int32)(Robot_Analyzers.Robo_Compass - 10);

                            if (Robot_Analyzers.Max_Compass >= 3000) Robot_Analyzers.Max_Compass = 3000;
                            if (Robot_Analyzers.Min_Compass <= -3000) Robot_Analyzers.Min_Compass = 3000;
                            if (Robot_Analyzers.Max_ABC >= 1000) Robot_Analyzers.Max_ABC = 1000;

                            if (Robot_Analyzers.Compass_input3_Max <= Robot_Analyzers.Robo_Input_Val3) Robot_Analyzers.Compass_input3_Max = Robot_Analyzers.Robo_Input_Val3 + 1;
                            //Compass_input3_Maxi
                            if (Robot_Analyzers.Compass_input3_Maxi < Robot_Analyzers.Compass_input3_Max)
                            {
                                Robot_Analyzers.Compass_input3_Maxi = Robot_Analyzers.Compass_input3_Max;
                                Y_SCope7.Dispatcher.Invoke(() => Y_SCope7.VisibleRange = new DoubleRange(0, Robot_Analyzers.Compass_input3_Max));
                            }

                            if (Robot_Analyzers.Max_ADCi < Robot_Analyzers.Max_ADC)
                            {
                                Robot_Analyzers.Max_ADCi = Robot_Analyzers.Max_ADC;
                                Y_SCope3.Dispatcher.Invoke(() => Y_SCope3.VisibleRange = new DoubleRange(0, Robot_Analyzers.Max_ADC));

                            }

                            if ((Robot_Analyzers.Max_Compassi < Robot_Analyzers.Max_Compass) || (Robot_Analyzers.Min_Compassi > Robot_Analyzers.Min_Compass))
                            {
                                Robot_Analyzers.Max_Compassi = Robot_Analyzers.Max_Compass;
                                Robot_Analyzers.Min_Compassi = Robot_Analyzers.Min_Compass;

                                Y_SCope5.Dispatcher.Invoke(() => Y_SCope5.VisibleRange = new DoubleRange(Robot_Analyzers.Min_Compass, Robot_Analyzers.Max_Compass));

                            }

                            if (Robot_Analyzers.Max_ABCi < Robot_Analyzers.Max_ABC)
                            {
                                Robot_Analyzers.Max_ABCi = Robot_Analyzers.Max_ABC;
                                Y_SCope6.Dispatcher.Invoke(() => Y_SCope6.VisibleRange = new DoubleRange(0, Robot_Analyzers.Max_ABC));

                            }
                        }

                        //}
                        //catch
                        //{

                        //}

                        RB_tb1.Dispatcher.Invoke(() => RB_tb1.Text = string.Concat(Robot_Analyzers.Robo_Restart));

                        if ((Robot_Analyzers.Robo_Restart < 20) && ((Robot_Analyzers.Robo_Sample % 10) == 0))
                        {
                            //RB_tb1.Dispatcher.Invoke(() => RB_tb1.Text = string.Concat(Robot_Analyzers.Robo_Restart));
                            RB_tb2.Dispatcher.Invoke(() => RB_tb2.Text = string.Concat(Robot_Analyzers.Robo_Encoder));
                            RB_tb3.Dispatcher.Invoke(() => RB_tb3.Text = string.Concat(Math.Round(Robot_Analyzers.Robo_Compass, 2)));

                            RB_tb4.Dispatcher.Invoke(() => RB_tb4.Text = string.Concat(Robot_Analyzers.Robo_duty[0]));
                            RB_tb5.Dispatcher.Invoke(() => RB_tb5.Text = string.Concat(Robot_Analyzers.Robot_View_Delay));
                            RB_tb6.Dispatcher.Invoke(() => RB_tb6.Text = string.Concat(Robot_Analyzers.Robo_duty[1]));
                            RB_tb7.Dispatcher.Invoke(() => RB_tb7.Text = string.Concat("0x", Robot_Analyzers.Robo_Input_Val.ToString("X2")));
                            RB_tb8.Dispatcher.Invoke(() => RB_tb8.Text = string.Concat(Robot_Analyzers.Robo_duty[2]));
                            RB_tb9.Dispatcher.Invoke(() => RB_tb9.Text = string.Concat("0x", Robot_Analyzers.Robo_Input_Val2.ToString("X2")));
                            RB_tb10.Dispatcher.Invoke(() => RB_tb10.Text = string.Concat(Robot_Analyzers.Robo_duty[3]));
                            RB_tb11.Dispatcher.Invoke(() => RB_tb11.Text = string.Concat(Robot_Analyzers.Robo_Input_Val3));

                            RB_tb12.Dispatcher.Invoke(() => RB_tb12.Text = string.Concat(Robot_Analyzers.Robo_A_Val));
                            RB_tb13.Dispatcher.Invoke(() => RB_tb13.Text = string.Concat(Robot_Analyzers.Robo_B_Val));
                            RB_tb14.Dispatcher.Invoke(() => RB_tb14.Text = string.Concat(Robot_Analyzers.Robo_C_Val));

                            RB_tb17.Dispatcher.Invoke(() => RB_tb17.Text = string.Concat(Robot_Analyzers.Robo_ADC1));
                            RB_tb18.Dispatcher.Invoke(() => RB_tb18.Text = string.Concat(Robot_Analyzers.Robo_ADC2));
                            RB_tb20.Dispatcher.Invoke(() => RB_tb20.Text = string.Concat(Robot_Analyzers.Robo_ADC3));
                        }
                    }

                    if (Robot_Analyzers.Clear_Request1 == 1)
                    {
                        Robot_Analyzers.Clear_Request1 = 0;

                        X_SCope4.Dispatcher.Invoke(() => X_SCope4.VisibleRange = new DoubleRange(0, 150));
                        X_SCope3.Dispatcher.Invoke(() => X_SCope3.VisibleRange = new DoubleRange(0, 150));
                        X_SCope5.Dispatcher.Invoke(() => X_SCope5.VisibleRange = new DoubleRange(0, 150));
                        X_SCope6.Dispatcher.Invoke(() => X_SCope6.VisibleRange = new DoubleRange(0, 150));
                        X_SCope7.Dispatcher.Invoke(() => X_SCope7.VisibleRange = new DoubleRange(0, 150));

                        S1_Line1i.Dispatcher.Invoke(() => S1_Line1i.X1 = 30);
                        S2_Line1i.Dispatcher.Invoke(() => S2_Line1i.X1 = 30);
                        S3_Line1i.Dispatcher.Invoke(() => S3_Line1i.X1 = 30);
                        S4_Line1i.Dispatcher.Invoke(() => S4_Line1i.X1 = 30);
                        S5_Line1i.Dispatcher.Invoke(() => S5_Line1i.X1 = 30);

                        //_xyDataSeries_rba = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb2 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb3 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb4 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb5 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb6 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb7 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb8 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb9 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb10 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb11 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb12 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb13 = new XyDataSeries<double>();

                        Robot_Analyzers._xyDataSeries_IO9 = new XyDataSeries<double>();

                        //Sensor_Compass.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = _xyDataSeries_rba);
                        Sensor_Duty1.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = Robot_Analyzers._xyDataSeries_rb2);
                        Sensor_Duty2.Dispatcher.Invoke(() => Sensor_Duty2.DataSeries = Robot_Analyzers._xyDataSeries_rb3);
                        Sensor_Duty3.Dispatcher.Invoke(() => Sensor_Duty3.DataSeries = Robot_Analyzers._xyDataSeries_rb4);
                        Sensor_Duty4.Dispatcher.Invoke(() => Sensor_Duty4.DataSeries = Robot_Analyzers._xyDataSeries_rb5);

                        Sensor_adcData1.Dispatcher.Invoke(() => Sensor_adcData1.DataSeries = Robot_Analyzers._xyDataSeries_rb6);
                        Sensor_adcData2.Dispatcher.Invoke(() => Sensor_adcData2.DataSeries = Robot_Analyzers._xyDataSeries_rb7);
                        Sensor_adcData3.Dispatcher.Invoke(() => Sensor_adcData3.DataSeries = Robot_Analyzers._xyDataSeries_rb8);

                        Sensor_compassData.Dispatcher.Invoke(() => Sensor_compassData.DataSeries = Robot_Analyzers._xyDataSeries_rb9);

                        Sensor_A_Data.Dispatcher.Invoke(() => Sensor_A_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb10);
                        Sensor_B_Data.Dispatcher.Invoke(() => Sensor_B_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb11);
                        Sensor_C_Data.Dispatcher.Invoke(() => Sensor_C_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb12);
                        Sensor_Encoder.Dispatcher.Invoke(() => Sensor_Encoder.DataSeries = Robot_Analyzers._xyDataSeries_rb13);

                        Sensor_lOGIC9.Dispatcher.Invoke(() => Sensor_lOGIC9.DataSeries = Robot_Analyzers._xyDataSeries_IO9);

                        Robot_Analyzers.Robo_Sample = 0;
                        Robot_Analyzers.Max_ADC = 100;
                        Robot_Analyzers.Max_Compass = 10;
                        Robot_Analyzers.Min_Compass = -10;
                        Robot_Analyzers.Max_ABC = 100;
                        Robot_Analyzers.Max_ADCi = 100;
                        Robot_Analyzers.Max_Compassi = 10;
                        Robot_Analyzers.Min_Compassi = -10;
                        Robot_Analyzers.Max_ABCi = 100;
                        Robot_Analyzers.Compass_input3_Maxi = 1.2;
                        Robot_Analyzers.Robot_Point = 0;

                        Y_SCope3.Dispatcher.Invoke(() => Y_SCope3.VisibleRange = new DoubleRange(0, 100));
                        Y_SCope5.Dispatcher.Invoke(() => Y_SCope5.VisibleRange = new DoubleRange(-10, 10));
                        Y_SCope6.Dispatcher.Invoke(() => Y_SCope6.VisibleRange = new DoubleRange(0, 100));
                        Y_SCope7.Dispatcher.Invoke(() => Y_SCope7.VisibleRange = new DoubleRange(0, 1.2));
                    }
                }
                catch
                {

                }
            }
        }

        public void Robo_Signal2()
        {
            Robot_Analyzers.Robot_Thread_Run = 2;

            Robot_Analyzers.connect = 0;
            Robot_Analyzers.Counters = 0;
            Robot_Analyzers.Counters2 = 0;

            Robot_Analyzers.Max_ADC = 0;
            Robot_Analyzers.Max_Compass = 0;
            Robot_Analyzers.Min_Compass = 0;
            Robot_Analyzers.Max_ABC = 100;

            Robot_Analyzers.Max_ADCi = 0;
            Robot_Analyzers.Max_Compassi = 0;
            Robot_Analyzers.Min_Compassi = 0;
            Robot_Analyzers.Max_ABCi = 100;

            //Sensor_Compass.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = _xyDataSeries_rba);
            Sensor_Duty1.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = Robot_Analyzers._xyDataSeries_rb2);
            Sensor_Duty2.Dispatcher.Invoke(() => Sensor_Duty2.DataSeries = Robot_Analyzers._xyDataSeries_rb3);
            Sensor_Duty3.Dispatcher.Invoke(() => Sensor_Duty3.DataSeries = Robot_Analyzers._xyDataSeries_rb4);
            Sensor_Duty4.Dispatcher.Invoke(() => Sensor_Duty4.DataSeries = Robot_Analyzers._xyDataSeries_rb5);

            Sensor_adcData1.Dispatcher.Invoke(() => Sensor_adcData1.DataSeries = Robot_Analyzers._xyDataSeries_rb6);
            Sensor_adcData2.Dispatcher.Invoke(() => Sensor_adcData2.DataSeries = Robot_Analyzers._xyDataSeries_rb7);
            Sensor_adcData3.Dispatcher.Invoke(() => Sensor_adcData3.DataSeries = Robot_Analyzers._xyDataSeries_rb8);
            Sensor_Encoder.Dispatcher.Invoke(() => Sensor_Encoder.DataSeries = Robot_Analyzers._xyDataSeries_rb13);

            Sensor_compassData.Dispatcher.Invoke(() => Sensor_compassData.DataSeries = Robot_Analyzers._xyDataSeries_rb9);

            Sensor_A_Data.Dispatcher.Invoke(() => Sensor_A_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb10);
            Sensor_B_Data.Dispatcher.Invoke(() => Sensor_B_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb11);
            Sensor_C_Data.Dispatcher.Invoke(() => Sensor_C_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb12);

            Sensor_lOGIC9.Dispatcher.Invoke(() => Sensor_lOGIC9.DataSeries = Robot_Analyzers._xyDataSeries_IO9);

            //UInt16 connect = 0;
            

            while (true)
            {
                try
                {
                    Robot_Analyzers.Robo_Frame2 = Rabit_Client.Receive(ref Rabit_Clienti);

                    Robot_Analyzers.Robo_Restart = Robot_Analyzers.Robo_Frame2[3];
                    Robot_Analyzers.Robo_Encoder = (Robot_Analyzers.Robo_Frame2[4] * 256 + Robot_Analyzers.Robo_Frame2[5]);
                    Robot_Analyzers.Robo_Compass = ((Robot_Analyzers.Robo_Frame2[6] * 256 + Robot_Analyzers.Robo_Frame2[7]) - 32768) / 10.0;

                    Robot_Analyzers.Robo_duty[0] = (Robot_Analyzers.Robo_Frame2[8] * 256 + Robot_Analyzers.Robo_Frame2[9]) - 255;
                    Robot_Analyzers.Robot_View_Delay = Robot_Analyzers.Robo_Frame2[10]; // delay 0 - 250
                    Robot_Analyzers.Robo_duty[1] = (Robot_Analyzers.Robo_Frame2[11] * 256 + Robot_Analyzers.Robo_Frame2[12]) - 255;
                    Robot_Analyzers.Robo_Input_Val = Robot_Analyzers.Robo_Frame2[13];
                    Robot_Analyzers.Robo_duty[2] = (Robot_Analyzers.Robo_Frame2[14] * 256 + Robot_Analyzers.Robo_Frame2[15]) - 255;
                    Robot_Analyzers.Robo_Input_Val2 = Robot_Analyzers.Robo_Frame2[16];
                    Robot_Analyzers.Robo_duty[3] = (Robot_Analyzers.Robo_Frame2[17] * 256 + Robot_Analyzers.Robo_Frame2[18]) - 255;
                    Robot_Analyzers.Robo_Input_Val3 = Robot_Analyzers.Robo_Frame2[19];

                    Robot_Analyzers.Robo_ADC1 = (uint)(Robot_Analyzers.Robo_Frame2[20] * 256 + Robot_Analyzers.Robo_Frame2[21]);
                    Robot_Analyzers.Robo_ADC2 = (uint)(Robot_Analyzers.Robo_Frame2[22] * 256 + Robot_Analyzers.Robo_Frame2[23]);
                    Robot_Analyzers.Robo_ADC3 = (uint)(Robot_Analyzers.Robo_Frame2[24] * 256 + Robot_Analyzers.Robo_Frame2[25]);

                    Robot_Analyzers.Robo_A_Val = (uint)(Robot_Analyzers.Robo_Frame2[26] * 256 + Robot_Analyzers.Robo_Frame2[27]);
                    Robot_Analyzers.Robo_B_Val = (uint)(Robot_Analyzers.Robo_Frame2[28] * 256 + Robot_Analyzers.Robo_Frame2[29]);
                    Robot_Analyzers.Robo_C_Val = (uint)(Robot_Analyzers.Robo_Frame2[30] * 256 + Robot_Analyzers.Robo_Frame2[31]);
                   
                    if (Robot_Analyzers.Robot_View_Delay < 10) Robot_Analyzers.Robot_View_Delay = 10;

                    if (Robot_Analyzers.Robo_Restart == 0)
                    {
                        Robot_Analyzers.Robot_Packet_Save_File = 0;
                    }
                    else if ((Robot_Analyzers.Robo_Restart >= 1) && (Robot_Analyzers.Robo_Restart < 20))
                    {
                        Robot_Analyzers.Robot_Packet_Save_File = 1;
                    }
                    else if (Robot_Analyzers.Robo_Restart == 21)
                    {
                        Robot_Analyzers.Robot_Packet_Save_File = 2;
                    }

                    Save_Packet2();

                    Robot_Analyzers.Robot_View_Clear = Robot_Analyzers.Robo_Restart;

                    if ((Robot_Analyzers.Robot_View_Cleari != Robot_Analyzers.Robot_View_Clear) && (Robot_Analyzers.Robot_View_Clear < 20))
                    {
                        Robot_Analyzers.Robot_View_Cleari = Robot_Analyzers.Robot_View_Clear;
                        Robot_Analyzers.Clear_Request1 = 1;
                    }

                    if (Robot_Analyzers.Robot_View_Stop == 0)
                    {
                        //if (Robot_Analyzers.Robot_View_Mode == 0)
                        //{
                        if ((Robot_Analyzers.Robo_Restart >= 1) && (Robot_Analyzers.Robo_Restart < 20))
                        {
                            Robot_Analyzers.Robo_Sample++;

                            if ((Robot_Analyzers.Robo_Sample % 5) == 0)
                            {
                                RB_tbx.Dispatcher.Invoke(() => RB_tbx.Text = string.Concat(Robot_Analyzers.Robo_Sample));
                                RB_tbx_time.Dispatcher.Invoke(() => RB_tbx_time.Text = string.Concat(Math.Round((Robot_Analyzers.Robo_Sample * Robot_Analyzers.Robot_View_Delay) / 1000.0, 2), "s"));
                            }

                            if (Robot_Analyzers.Robo_Sample > 20000) Robot_Analyzers.Clear_Request1 = 1;

                            if ((Robot_Analyzers.Robo_Sample >= 150) && (Robot_Analyzers.Robo_Sample % 150 == 0))
                            {
                                X_SCope4.Dispatcher.Invoke(() => X_SCope4.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                X_SCope3.Dispatcher.Invoke(() => X_SCope3.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                X_SCope5.Dispatcher.Invoke(() => X_SCope5.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                X_SCope6.Dispatcher.Invoke(() => X_SCope6.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                X_SCope7.Dispatcher.Invoke(() => X_SCope7.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                            }
                        }


                        ////try
                        ////{
                        //if (Robot_Analyzers.Robot_View_Mode == 0)
                        //{
                        if ((Robot_Analyzers.Robo_Restart >= 1) && (Robot_Analyzers.Robo_Restart < 20))
                        {
                            //_xyDataSeries_rba.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Compass);
                            Robot_Analyzers._xyDataSeries_rb2.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[0]);
                            Robot_Analyzers._xyDataSeries_rb3.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[1]);
                            Robot_Analyzers._xyDataSeries_rb4.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[2]);
                            Robot_Analyzers._xyDataSeries_rb5.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[3]);
                            Robot_Analyzers._xyDataSeries_rb6.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_ADC1);
                            Robot_Analyzers._xyDataSeries_rb7.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_ADC2);
                            Robot_Analyzers._xyDataSeries_rb8.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_ADC3);
                            Robot_Analyzers._xyDataSeries_rb9.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Compass);
                            Robot_Analyzers._xyDataSeries_rb10.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_A_Val);
                            Robot_Analyzers._xyDataSeries_rb11.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_B_Val);
                            Robot_Analyzers._xyDataSeries_rb12.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_C_Val);
                            Robot_Analyzers._xyDataSeries_rb13.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Encoder);

                            Robot_Analyzers._xyDataSeries_IO9.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Input_Val3);

                            if (Robot_Analyzers.Max_ADC <= Robot_Analyzers.Robo_ADC1) Robot_Analyzers.Max_ADC = Robot_Analyzers.Robo_ADC1 + 2000;
                            if (Robot_Analyzers.Max_ADC <= Robot_Analyzers.Robo_ADC2) Robot_Analyzers.Max_ADC = Robot_Analyzers.Robo_ADC2 + 2000;
                            if (Robot_Analyzers.Max_ADC <= Robot_Analyzers.Robo_ADC3) Robot_Analyzers.Max_ADC = Robot_Analyzers.Robo_ADC3 + 2000;

                            if (Robot_Analyzers.Max_Compass <= Robot_Analyzers.Robo_A_Val) Robot_Analyzers.Max_Compass = (int)(Robot_Analyzers.Robo_A_Val + 10);

                            if (Robot_Analyzers.Max_ABC <= Robot_Analyzers.Robo_B_Val) Robot_Analyzers.Max_ABC = Robot_Analyzers.Robo_B_Val + 100;
                            if (Robot_Analyzers.Max_ABC <= Robot_Analyzers.Robo_C_Val) Robot_Analyzers.Max_ABC = Robot_Analyzers.Robo_C_Val + 100;

                            if (Robot_Analyzers.Robo_Compass >= Robot_Analyzers.Max_Compass) Robot_Analyzers.Max_Compass = (Int32)(Robot_Analyzers.Robo_Compass + 10);
                            if (Robot_Analyzers.Robo_Compass <= Robot_Analyzers.Min_Compass) Robot_Analyzers.Min_Compass = (Int32)(Robot_Analyzers.Robo_Compass - 10);

                            if (Robot_Analyzers.Max_Compass >= 3000) Robot_Analyzers.Max_Compass = 3000;
                            if (Robot_Analyzers.Min_Compass <= -3000) Robot_Analyzers.Min_Compass = 3000;
                            if (Robot_Analyzers.Max_ABC >= 1000) Robot_Analyzers.Max_ABC = 1000;

                            if (Robot_Analyzers.Compass_input3_Max <= Robot_Analyzers.Robo_Input_Val3) Robot_Analyzers.Compass_input3_Max = Robot_Analyzers.Robo_Input_Val3 + 1;
                            //Compass_input3_Maxi
                            if (Robot_Analyzers.Compass_input3_Maxi < Robot_Analyzers.Compass_input3_Max)
                            {
                                Robot_Analyzers.Compass_input3_Maxi = Robot_Analyzers.Compass_input3_Max;
                                Y_SCope7.Dispatcher.Invoke(() => Y_SCope7.VisibleRange = new DoubleRange(0, Robot_Analyzers.Compass_input3_Max));
                            }

                            if (Robot_Analyzers.Max_ADCi < Robot_Analyzers.Max_ADC)
                            {
                                Robot_Analyzers.Max_ADCi = Robot_Analyzers.Max_ADC;
                                Y_SCope3.Dispatcher.Invoke(() => Y_SCope3.VisibleRange = new DoubleRange(0, Robot_Analyzers.Max_ADC));

                            }

                            if ((Robot_Analyzers.Max_Compassi < Robot_Analyzers.Max_Compass) || (Robot_Analyzers.Min_Compassi > Robot_Analyzers.Min_Compass))
                            {
                                Robot_Analyzers.Max_Compassi = Robot_Analyzers.Max_Compass;
                                Robot_Analyzers.Min_Compassi = Robot_Analyzers.Min_Compass;

                                Y_SCope5.Dispatcher.Invoke(() => Y_SCope5.VisibleRange = new DoubleRange(Robot_Analyzers.Min_Compass, Robot_Analyzers.Max_Compass));

                            }

                            if (Robot_Analyzers.Max_ABCi < Robot_Analyzers.Max_ABC)
                            {
                                Robot_Analyzers.Max_ABCi = Robot_Analyzers.Max_ABC;
                                Y_SCope6.Dispatcher.Invoke(() => Y_SCope6.VisibleRange = new DoubleRange(0, Robot_Analyzers.Max_ABC));

                            }
                        }

                        //}
                        //catch
                        //{

                        //}

                        RB_tb1.Dispatcher.Invoke(() => RB_tb1.Text = string.Concat(Robot_Analyzers.Robo_Restart));

                        if ((Robot_Analyzers.Robo_Restart < 20) && ((Robot_Analyzers.Robo_Sample % 10) == 0))
                        {
                            //RB_tb1.Dispatcher.Invoke(() => RB_tb1.Text = string.Concat(Robot_Analyzers.Robo_Restart));
                            RB_tb2.Dispatcher.Invoke(() => RB_tb2.Text = string.Concat(Robot_Analyzers.Robo_Encoder));
                            RB_tb3.Dispatcher.Invoke(() => RB_tb3.Text = string.Concat(Math.Round(Robot_Analyzers.Robo_Compass, 2)));

                            RB_tb4.Dispatcher.Invoke(() => RB_tb4.Text = string.Concat(Robot_Analyzers.Robo_duty[0]));
                            RB_tb5.Dispatcher.Invoke(() => RB_tb5.Text = string.Concat(Robot_Analyzers.Robot_View_Delay));
                            RB_tb6.Dispatcher.Invoke(() => RB_tb6.Text = string.Concat(Robot_Analyzers.Robo_duty[1]));
                            RB_tb7.Dispatcher.Invoke(() => RB_tb7.Text = string.Concat("0x", Robot_Analyzers.Robo_Input_Val.ToString("X2")));
                            RB_tb8.Dispatcher.Invoke(() => RB_tb8.Text = string.Concat(Robot_Analyzers.Robo_duty[2]));
                            RB_tb9.Dispatcher.Invoke(() => RB_tb9.Text = string.Concat("0x", Robot_Analyzers.Robo_Input_Val2.ToString("X2")));
                            RB_tb10.Dispatcher.Invoke(() => RB_tb10.Text = string.Concat(Robot_Analyzers.Robo_duty[3]));
                            RB_tb11.Dispatcher.Invoke(() => RB_tb11.Text = string.Concat(Robot_Analyzers.Robo_Input_Val3));

                            RB_tb12.Dispatcher.Invoke(() => RB_tb12.Text = string.Concat(Robot_Analyzers.Robo_A_Val));
                            RB_tb13.Dispatcher.Invoke(() => RB_tb13.Text = string.Concat(Robot_Analyzers.Robo_B_Val));
                            RB_tb14.Dispatcher.Invoke(() => RB_tb14.Text = string.Concat(Robot_Analyzers.Robo_C_Val));

                            RB_tb17.Dispatcher.Invoke(() => RB_tb17.Text = string.Concat(Robot_Analyzers.Robo_ADC1));
                            RB_tb18.Dispatcher.Invoke(() => RB_tb18.Text = string.Concat(Robot_Analyzers.Robo_ADC2));
                            RB_tb20.Dispatcher.Invoke(() => RB_tb20.Text = string.Concat(Robot_Analyzers.Robo_ADC3));
                        }
                    }

                    if (Robot_Analyzers.Clear_Request1 == 1)
                    {
                        Robot_Analyzers.Clear_Request1 = 0;

                        X_SCope4.Dispatcher.Invoke(() => X_SCope4.VisibleRange = new DoubleRange(0, 150));
                        X_SCope3.Dispatcher.Invoke(() => X_SCope3.VisibleRange = new DoubleRange(0, 150));
                        X_SCope5.Dispatcher.Invoke(() => X_SCope5.VisibleRange = new DoubleRange(0, 150));
                        X_SCope6.Dispatcher.Invoke(() => X_SCope6.VisibleRange = new DoubleRange(0, 150));
                        X_SCope7.Dispatcher.Invoke(() => X_SCope7.VisibleRange = new DoubleRange(0, 150));

                        S1_Line1i.Dispatcher.Invoke(() => S1_Line1i.X1 = 30);
                        S2_Line1i.Dispatcher.Invoke(() => S2_Line1i.X1 = 30);
                        S3_Line1i.Dispatcher.Invoke(() => S3_Line1i.X1 = 30);
                        S4_Line1i.Dispatcher.Invoke(() => S4_Line1i.X1 = 30);
                        S5_Line1i.Dispatcher.Invoke(() => S5_Line1i.X1 = 30);

                        //_xyDataSeries_rba = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb2 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb3 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb4 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb5 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb6 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb7 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb8 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb9 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb10 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb11 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb12 = new XyDataSeries<double>();
                        Robot_Analyzers._xyDataSeries_rb13 = new XyDataSeries<double>();

                        Robot_Analyzers._xyDataSeries_IO9 = new XyDataSeries<double>();

                        //Sensor_Compass.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = _xyDataSeries_rba);
                        Sensor_Duty1.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = Robot_Analyzers._xyDataSeries_rb2);
                        Sensor_Duty2.Dispatcher.Invoke(() => Sensor_Duty2.DataSeries = Robot_Analyzers._xyDataSeries_rb3);
                        Sensor_Duty3.Dispatcher.Invoke(() => Sensor_Duty3.DataSeries = Robot_Analyzers._xyDataSeries_rb4);
                        Sensor_Duty4.Dispatcher.Invoke(() => Sensor_Duty4.DataSeries = Robot_Analyzers._xyDataSeries_rb5);

                        Sensor_adcData1.Dispatcher.Invoke(() => Sensor_adcData1.DataSeries = Robot_Analyzers._xyDataSeries_rb6);
                        Sensor_adcData2.Dispatcher.Invoke(() => Sensor_adcData2.DataSeries = Robot_Analyzers._xyDataSeries_rb7);
                        Sensor_adcData3.Dispatcher.Invoke(() => Sensor_adcData3.DataSeries = Robot_Analyzers._xyDataSeries_rb8);

                        Sensor_compassData.Dispatcher.Invoke(() => Sensor_compassData.DataSeries = Robot_Analyzers._xyDataSeries_rb9);

                        Sensor_A_Data.Dispatcher.Invoke(() => Sensor_A_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb10);
                        Sensor_B_Data.Dispatcher.Invoke(() => Sensor_B_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb11);
                        Sensor_C_Data.Dispatcher.Invoke(() => Sensor_C_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb12);
                        Sensor_Encoder.Dispatcher.Invoke(() => Sensor_Encoder.DataSeries = Robot_Analyzers._xyDataSeries_rb13);

                        Sensor_lOGIC9.Dispatcher.Invoke(() => Sensor_lOGIC9.DataSeries = Robot_Analyzers._xyDataSeries_IO9);

                        Robot_Analyzers.Robo_Sample = 0;
                        Robot_Analyzers.Max_ADC = 100;
                        Robot_Analyzers.Max_Compass = 10;
                        Robot_Analyzers.Min_Compass = -10;
                        Robot_Analyzers.Max_ABC = 100;
                        Robot_Analyzers.Max_ADCi = 100;
                        Robot_Analyzers.Max_Compassi = 10;
                        Robot_Analyzers.Min_Compassi = -10;
                        Robot_Analyzers.Max_ABCi = 100;
                        Robot_Analyzers.Compass_input3_Maxi = 1.2;
                        Robot_Analyzers.Robot_Point = 0;

                        Y_SCope3.Dispatcher.Invoke(() => Y_SCope3.VisibleRange = new DoubleRange(0, 100));
                        Y_SCope5.Dispatcher.Invoke(() => Y_SCope5.VisibleRange = new DoubleRange(-10, 10));
                        Y_SCope6.Dispatcher.Invoke(() => Y_SCope6.VisibleRange = new DoubleRange(0, 100));
                        Y_SCope7.Dispatcher.Invoke(() => Y_SCope7.VisibleRange = new DoubleRange(0, 1.2));
                    }

                }
                catch
                {

                }
            }
        }

        public void Robo_Analyse()
        {
            // kiểm tra kết nối PC1
            //UInt16 Lat0 = 0;
            UInt16 connect = 0;
            UInt16 Counters = 0;
            UInt16 Counters2 = 0;

            UInt32 Max_ADC = 0;
            Int32 Max_Compass = 0;
            Int32 Min_Compass = 0;
            UInt32 Max_ABC = 100;

            UInt32 Max_ADCi = 0;
            Int32 Max_Compassi = 0;
            Int32 Min_Compassi = 0;
            UInt32 Max_ABCi = 100;

            //Sensor_Compass.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = _xyDataSeries_rba);
            Sensor_Duty1.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = Robot_Analyzers._xyDataSeries_rb2);
            Sensor_Duty2.Dispatcher.Invoke(() => Sensor_Duty2.DataSeries = Robot_Analyzers._xyDataSeries_rb3);
            Sensor_Duty3.Dispatcher.Invoke(() => Sensor_Duty3.DataSeries = Robot_Analyzers._xyDataSeries_rb4);
            Sensor_Duty4.Dispatcher.Invoke(() => Sensor_Duty4.DataSeries = Robot_Analyzers._xyDataSeries_rb5);

            Sensor_adcData1.Dispatcher.Invoke(() => Sensor_adcData1.DataSeries = Robot_Analyzers._xyDataSeries_rb6);
            Sensor_adcData2.Dispatcher.Invoke(() => Sensor_adcData2.DataSeries = Robot_Analyzers._xyDataSeries_rb7);
            Sensor_adcData3.Dispatcher.Invoke(() => Sensor_adcData3.DataSeries = Robot_Analyzers._xyDataSeries_rb8);
            Sensor_Encoder.Dispatcher.Invoke(() => Sensor_Encoder.DataSeries = Robot_Analyzers._xyDataSeries_rb13);

            Sensor_compassData.Dispatcher.Invoke(() => Sensor_compassData.DataSeries = Robot_Analyzers._xyDataSeries_rb9);

            Sensor_A_Data.Dispatcher.Invoke(() => Sensor_A_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb10);
            Sensor_B_Data.Dispatcher.Invoke(() => Sensor_B_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb11);
            Sensor_C_Data.Dispatcher.Invoke(() => Sensor_C_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb12);

            Sensor_lOGIC9.Dispatcher.Invoke(() => Sensor_lOGIC9.DataSeries = Robot_Analyzers._xyDataSeries_IO9);

            while (true)
            {
                try
                {
                    //Robo_Frame = Robo_Client.Receive(ref Robo_Clienti);

                    if (Robot_Analyzers.Robot_Analyse == 1)
                    {
                        Robot_Analyzers.Robot_Analyse = 0;
    
                        // ROBOT THỎ
                        if (Robot_Analyzers.Robot_View_Mode == 1)
                        {
                            Robot_Analyzers.Robo_Restart = Robot_Analyzers.Robo_Frame2[3];
                            Robot_Analyzers.Robo_Encoder = (Robot_Analyzers.Robo_Frame2[4] * 256 + Robot_Analyzers.Robo_Frame2[5]);
                            Robot_Analyzers.Robo_Compass = ((Robot_Analyzers.Robo_Frame2[6] * 256 + Robot_Analyzers.Robo_Frame2[7]) - 32768) / 10.0;

                            Robot_Analyzers.Robo_duty[0] = (Robot_Analyzers.Robo_Frame2[8] * 256 + Robot_Analyzers.Robo_Frame2[9]) - 255;
                            Robot_Analyzers.Robot_View_Delay = Robot_Analyzers.Robo_Frame2[10]; // delay 0 - 250
                            Robot_Analyzers.Robo_duty[1] = (Robot_Analyzers.Robo_Frame2[11] * 256 + Robot_Analyzers.Robo_Frame2[12]) - 255;
                            Robot_Analyzers.Robo_Input_Val = Robot_Analyzers.Robo_Frame2[13];
                            Robot_Analyzers.Robo_duty[2] = (Robot_Analyzers.Robo_Frame2[14] * 256 + Robot_Analyzers.Robo_Frame2[15]) - 255;
                            Robot_Analyzers.Robo_Input_Val2 = Robot_Analyzers.Robo_Frame2[16];
                            Robot_Analyzers.Robo_duty[3] = (Robot_Analyzers.Robo_Frame2[17] * 256 + Robot_Analyzers.Robo_Frame2[18]) - 255;
                            Robot_Analyzers.Robo_Input_Val3 = Robot_Analyzers.Robo_Frame2[19];

                            Robot_Analyzers.Robo_ADC1 = (uint)(Robot_Analyzers.Robo_Frame2[20] * 256 + Robot_Analyzers.Robo_Frame2[21]);
                            Robot_Analyzers.Robo_ADC2 = (uint)(Robot_Analyzers.Robo_Frame2[22] * 256 + Robot_Analyzers.Robo_Frame2[23]);
                            Robot_Analyzers.Robo_ADC3 = (uint)(Robot_Analyzers.Robo_Frame2[24] * 256 + Robot_Analyzers.Robo_Frame2[25]);

                            Robot_Analyzers.Robo_A_Val = (uint)(Robot_Analyzers.Robo_Frame2[26] * 256 + Robot_Analyzers.Robo_Frame2[27]);
                            Robot_Analyzers.Robo_B_Val = (uint)(Robot_Analyzers.Robo_Frame2[28] * 256 + Robot_Analyzers.Robo_Frame2[29]);
                            Robot_Analyzers.Robo_C_Val = (uint)(Robot_Analyzers.Robo_Frame2[30] * 256 + Robot_Analyzers.Robo_Frame2[31]);

                        }
                        else
                        {
                            Robot_Analyzers.Robo_Restart = Robot_Analyzers.Robo_Frame[3];
                            Robot_Analyzers.Robo_Encoder = (Robot_Analyzers.Robo_Frame[4] * 256 + Robot_Analyzers.Robo_Frame[5]);
                            Robot_Analyzers.Robo_Compass = ((Robot_Analyzers.Robo_Frame[6] * 256 + Robot_Analyzers.Robo_Frame[7]) - 32768) / 10.0;

                            Robot_Analyzers.Robo_duty[0] = (Robot_Analyzers.Robo_Frame[8] * 256 + Robot_Analyzers.Robo_Frame[9]) - 255;
                            Robot_Analyzers.Robot_View_Delay = Robot_Analyzers.Robo_Frame[10];
                            Robot_Analyzers.Robo_duty[1] = (Robot_Analyzers.Robo_Frame[11] * 256 + Robot_Analyzers.Robo_Frame[12]) - 255;
                            Robot_Analyzers.Robo_Input_Val = Robot_Analyzers.Robo_Frame[13];
                            Robot_Analyzers.Robo_duty[2] = (Robot_Analyzers.Robo_Frame[14] * 256 + Robot_Analyzers.Robo_Frame[15]) - 255;
                            Robot_Analyzers.Robo_Input_Val2 = Robot_Analyzers.Robo_Frame[16];
                            Robot_Analyzers.Robo_duty[3] = (Robot_Analyzers.Robo_Frame[17] * 256 + Robot_Analyzers.Robo_Frame[18]) - 255;
                            Robot_Analyzers.Robo_Input_Val3 = Robot_Analyzers.Robo_Frame[19];

                            Robot_Analyzers.Robo_ADC1 = (uint)(Robot_Analyzers.Robo_Frame[20] * 256 + Robot_Analyzers.Robo_Frame[21]);
                            Robot_Analyzers.Robo_ADC2 = (uint)(Robot_Analyzers.Robo_Frame[22] * 256 + Robot_Analyzers.Robo_Frame[23]);
                            Robot_Analyzers.Robo_ADC3 = (uint)(Robot_Analyzers.Robo_Frame[24] * 256 + Robot_Analyzers.Robo_Frame[25]);

                            if (Robot_Analyzers.Robot_View_Delay < 10) Robot_Analyzers.Robot_View_Delay = 10;

                            Robot_Analyzers.Robo_A_Val = (uint)(Robot_Analyzers.Robo_Frame[26] * 256 + Robot_Analyzers.Robo_Frame[27]);
                            Robot_Analyzers.Robo_B_Val = (uint)(Robot_Analyzers.Robo_Frame[28] * 256 + Robot_Analyzers.Robo_Frame[29]);
                            Robot_Analyzers.Robo_C_Val = (uint)(Robot_Analyzers.Robo_Frame[30] * 256 + Robot_Analyzers.Robo_Frame[31]);

                        }

                        Robot_Analyzers.Robot_View_Clear = Robot_Analyzers.Robo_Restart;

                        if((Robot_Analyzers.Robot_View_Cleari != Robot_Analyzers.Robot_View_Clear) && (Robot_Analyzers.Robot_View_Clear < 20))
                        {
                            Robot_Analyzers.Robot_View_Cleari = Robot_Analyzers.Robot_View_Clear;
                            Robot_Analyzers.Clear_Request1 = 1;
                        }    

                        if (Robot_Analyzers.Robot_View_Stop == 0)
                        {
                            //if (Robot_Analyzers.Robot_View_Mode == 0)
                            //{
                                if ((Robot_Analyzers.Robo_Restart >= 1) && (Robot_Analyzers.Robo_Restart < 20))
                                {
                                    Robot_Analyzers.Robo_Sample++;

                                if ((Robot_Analyzers.Robo_Sample % 5) == 0)
                                {
                                    RB_tbx.Dispatcher.Invoke(() => RB_tbx.Text = string.Concat(Robot_Analyzers.Robo_Sample));
                                    RB_tbx_time.Dispatcher.Invoke(() => RB_tbx_time.Text = string.Concat(Math.Round((Robot_Analyzers.Robo_Sample * Robot_Analyzers.Robot_View_Delay) / 1000.0, 2), "s"));
                                }

                                    if (Robot_Analyzers.Robo_Sample > 20000) Robot_Analyzers.Clear_Request1 = 1;

                                    if ((Robot_Analyzers.Robo_Sample >= 150) && (Robot_Analyzers.Robo_Sample % 150 == 0))
                                    {
                                        X_SCope4.Dispatcher.Invoke(() => X_SCope4.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                        X_SCope3.Dispatcher.Invoke(() => X_SCope3.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                        X_SCope5.Dispatcher.Invoke(() => X_SCope5.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                        X_SCope6.Dispatcher.Invoke(() => X_SCope6.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                        X_SCope7.Dispatcher.Invoke(() => X_SCope7.VisibleRange = new DoubleRange(0, Robot_Analyzers.Robo_Sample + 150));
                                    }
                                }


                            try
                            {
                                //if (Robot_Analyzers.Robot_View_Mode == 0)
                                //{
                                    if ((Robot_Analyzers.Robo_Restart >= 1) && (Robot_Analyzers.Robo_Restart < 20))
                                    {
                                        //_xyDataSeries_rba.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Compass);
                                        Robot_Analyzers._xyDataSeries_rb2.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[0]);
                                        Robot_Analyzers._xyDataSeries_rb3.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[1]);
                                        Robot_Analyzers._xyDataSeries_rb4.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[2]);
                                        Robot_Analyzers._xyDataSeries_rb5.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_duty[3]);
                                        Robot_Analyzers._xyDataSeries_rb6.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_ADC1);
                                        Robot_Analyzers._xyDataSeries_rb7.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_ADC2);
                                        Robot_Analyzers._xyDataSeries_rb8.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_ADC3);
                                        Robot_Analyzers._xyDataSeries_rb9.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Compass);
                                        Robot_Analyzers._xyDataSeries_rb10.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_A_Val);
                                        Robot_Analyzers._xyDataSeries_rb11.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_B_Val);
                                        Robot_Analyzers._xyDataSeries_rb12.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_C_Val);
                                        Robot_Analyzers._xyDataSeries_rb13.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Encoder);

                                       Robot_Analyzers._xyDataSeries_IO9.Append(Robot_Analyzers.Robo_Sample, Robot_Analyzers.Robo_Input_Val3);

                                        if (Max_ADC <= Robot_Analyzers.Robo_ADC1) Max_ADC = Robot_Analyzers.Robo_ADC1 + 2000;
                                        if (Max_ADC <= Robot_Analyzers.Robo_ADC2) Max_ADC = Robot_Analyzers.Robo_ADC2 + 2000;
                                        if (Max_ADC <= Robot_Analyzers.Robo_ADC3) Max_ADC = Robot_Analyzers.Robo_ADC3 + 2000;

                                        if (Max_Compass <= Robot_Analyzers.Robo_A_Val) Max_Compass = (int)(Robot_Analyzers.Robo_A_Val + 10);

                                        if (Max_ABC <= Robot_Analyzers.Robo_B_Val) Max_ABC = Robot_Analyzers.Robo_B_Val + 100;
                                        if (Max_ABC <= Robot_Analyzers.Robo_C_Val) Max_ABC = Robot_Analyzers.Robo_C_Val + 100;

                                        if (Robot_Analyzers.Robo_Compass >= Max_Compass) Max_Compass = (Int32)(Robot_Analyzers.Robo_Compass + 10);
                                        if (Robot_Analyzers.Robo_Compass <= Min_Compass) Min_Compass = (Int32)(Robot_Analyzers.Robo_Compass - 10);

                                        if (Max_Compass >= 3000) Max_Compass = 3000;
                                        if (Min_Compass <= -3000) Min_Compass = 3000;
                                        if (Max_ABC >= 1000) Max_ABC = 1000;

                                        if (Robot_Analyzers.Compass_input3_Max <= Robot_Analyzers.Robo_Input_Val3) Robot_Analyzers.Compass_input3_Max = Robot_Analyzers.Robo_Input_Val3 + 1;
                                        //Compass_input3_Maxi
                                        if (Robot_Analyzers.Compass_input3_Maxi < Robot_Analyzers.Compass_input3_Max)
                                        {
                                            Robot_Analyzers.Compass_input3_Maxi = Robot_Analyzers.Compass_input3_Max;
                                            Y_SCope7.Dispatcher.Invoke(() => Y_SCope7.VisibleRange = new DoubleRange(0, Robot_Analyzers.Compass_input3_Max));
                                        }
      
                                        if (Max_ADCi < Max_ADC)
                                        {
                                            Max_ADCi = Max_ADC;
                                            Y_SCope3.Dispatcher.Invoke(() => Y_SCope3.VisibleRange = new DoubleRange(0, Max_ADC));

                                        }

                                        if ((Max_Compassi < Max_Compass) || (Min_Compassi > Min_Compass))
                                        {
                                            Max_Compassi = Max_Compass;
                                            Min_Compassi = Min_Compass;

                                            Y_SCope5.Dispatcher.Invoke(() => Y_SCope5.VisibleRange = new DoubleRange(Min_Compass, Max_Compass));

                                        }

                                        if (Max_ABCi < Max_ABC)
                                        {
                                            Max_ABCi = Max_ABC;
                                            Y_SCope6.Dispatcher.Invoke(() => Y_SCope6.VisibleRange = new DoubleRange(0, Max_ABC));

                                        }
                                    }
  
                            }
                            catch
                            {

                            }

                            RB_tb1.Dispatcher.Invoke(() => RB_tb1.Text = string.Concat(Robot_Analyzers.Robo_Restart));

                            if ((Robot_Analyzers.Robo_Restart < 20)&&((Robot_Analyzers.Robo_Sample % 10) == 0))
                            {
                                //RB_tb1.Dispatcher.Invoke(() => RB_tb1.Text = string.Concat(Robot_Analyzers.Robo_Restart));
                                RB_tb2.Dispatcher.Invoke(() => RB_tb2.Text = string.Concat(Robot_Analyzers.Robo_Encoder));
                                RB_tb3.Dispatcher.Invoke(() => RB_tb3.Text = string.Concat(Math.Round(Robot_Analyzers.Robo_Compass, 2)));

                                RB_tb4.Dispatcher.Invoke(() => RB_tb4.Text = string.Concat(Robot_Analyzers.Robo_duty[0]));
                                RB_tb5.Dispatcher.Invoke(() => RB_tb5.Text = string.Concat(Robot_Analyzers.Robot_View_Delay));
                                RB_tb6.Dispatcher.Invoke(() => RB_tb6.Text = string.Concat(Robot_Analyzers.Robo_duty[1]));
                                RB_tb7.Dispatcher.Invoke(() => RB_tb7.Text = string.Concat("0x", Robot_Analyzers.Robo_Input_Val.ToString("X2")));
                                RB_tb8.Dispatcher.Invoke(() => RB_tb8.Text = string.Concat(Robot_Analyzers.Robo_duty[2]));
                                RB_tb9.Dispatcher.Invoke(() => RB_tb9.Text = string.Concat("0x", Robot_Analyzers.Robo_Input_Val2.ToString("X2")));
                                RB_tb10.Dispatcher.Invoke(() => RB_tb10.Text = string.Concat(Robot_Analyzers.Robo_duty[3]));
                                RB_tb11.Dispatcher.Invoke(() => RB_tb11.Text = string.Concat(Robot_Analyzers.Robo_Input_Val3));

                                RB_tb12.Dispatcher.Invoke(() => RB_tb12.Text = string.Concat(Robot_Analyzers.Robo_A_Val));
                                RB_tb13.Dispatcher.Invoke(() => RB_tb13.Text = string.Concat(Robot_Analyzers.Robo_B_Val));
                                RB_tb14.Dispatcher.Invoke(() => RB_tb14.Text = string.Concat(Robot_Analyzers.Robo_C_Val));

                                RB_tb17.Dispatcher.Invoke(() => RB_tb17.Text = string.Concat(Robot_Analyzers.Robo_ADC1));
                                RB_tb18.Dispatcher.Invoke(() => RB_tb18.Text = string.Concat(Robot_Analyzers.Robo_ADC2));
                                RB_tb20.Dispatcher.Invoke(() => RB_tb20.Text = string.Concat(Robot_Analyzers.Robo_ADC3));
                            }
                        }

                        if (Robot_Analyzers.Clear_Request1 == 1)
                        {
                            Robot_Analyzers.Clear_Request1 = 0;

                            X_SCope4.Dispatcher.Invoke(() => X_SCope4.VisibleRange = new DoubleRange(0, 150));
                            X_SCope3.Dispatcher.Invoke(() => X_SCope3.VisibleRange = new DoubleRange(0, 150));
                            X_SCope5.Dispatcher.Invoke(() => X_SCope5.VisibleRange = new DoubleRange(0, 150));
                            X_SCope6.Dispatcher.Invoke(() => X_SCope6.VisibleRange = new DoubleRange(0, 150));
                            X_SCope7.Dispatcher.Invoke(() => X_SCope7.VisibleRange = new DoubleRange(0, 150));

                            S1_Line1i.Dispatcher.Invoke(() => S1_Line1i.X1 = 30);
                            S2_Line1i.Dispatcher.Invoke(() => S2_Line1i.X1 = 30);
                            S3_Line1i.Dispatcher.Invoke(() => S3_Line1i.X1 = 30);
                            S4_Line1i.Dispatcher.Invoke(() => S4_Line1i.X1 = 30);
                            S5_Line1i.Dispatcher.Invoke(() => S5_Line1i.X1 = 30);

                            //_xyDataSeries_rba = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb2 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb3 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb4 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb5 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb6 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb7 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb8 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb9 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb10 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb11 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb12 = new XyDataSeries<double>();
                            Robot_Analyzers._xyDataSeries_rb13 = new XyDataSeries<double>();

                            Robot_Analyzers._xyDataSeries_IO9 = new XyDataSeries<double>();

                            //Sensor_Compass.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = _xyDataSeries_rba);
                            Sensor_Duty1.Dispatcher.Invoke(() => Sensor_Duty1.DataSeries = Robot_Analyzers._xyDataSeries_rb2);
                            Sensor_Duty2.Dispatcher.Invoke(() => Sensor_Duty2.DataSeries = Robot_Analyzers._xyDataSeries_rb3);
                            Sensor_Duty3.Dispatcher.Invoke(() => Sensor_Duty3.DataSeries = Robot_Analyzers._xyDataSeries_rb4);
                            Sensor_Duty4.Dispatcher.Invoke(() => Sensor_Duty4.DataSeries = Robot_Analyzers._xyDataSeries_rb5);

                            Sensor_adcData1.Dispatcher.Invoke(() => Sensor_adcData1.DataSeries = Robot_Analyzers._xyDataSeries_rb6);
                            Sensor_adcData2.Dispatcher.Invoke(() => Sensor_adcData2.DataSeries = Robot_Analyzers._xyDataSeries_rb7);
                            Sensor_adcData3.Dispatcher.Invoke(() => Sensor_adcData3.DataSeries = Robot_Analyzers._xyDataSeries_rb8);

                            Sensor_compassData.Dispatcher.Invoke(() => Sensor_compassData.DataSeries = Robot_Analyzers._xyDataSeries_rb9);

                            Sensor_A_Data.Dispatcher.Invoke(() => Sensor_A_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb10);
                            Sensor_B_Data.Dispatcher.Invoke(() => Sensor_B_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb11);
                            Sensor_C_Data.Dispatcher.Invoke(() => Sensor_C_Data.DataSeries = Robot_Analyzers._xyDataSeries_rb12);
                            Sensor_Encoder.Dispatcher.Invoke(() => Sensor_Encoder.DataSeries = Robot_Analyzers._xyDataSeries_rb13);

                            Sensor_lOGIC9.Dispatcher.Invoke(() => Sensor_lOGIC9.DataSeries = Robot_Analyzers._xyDataSeries_IO9);

                            Robot_Analyzers.Robo_Sample = 0;
                            Max_ADC = 100;
                            Max_Compass = 10;
                            Min_Compass = -10;
                            Max_ABC = 100;
                            Max_ADCi = 100;
                            Max_Compassi = 10;
                            Min_Compassi = -10;
                            Max_ABCi = 100;
                            Robot_Analyzers.Compass_input3_Maxi = 1.2;
                            Robot_Analyzers.Robot_Point = 0;

                            Y_SCope3.Dispatcher.Invoke(() => Y_SCope3.VisibleRange = new DoubleRange(0, 100));
                            Y_SCope5.Dispatcher.Invoke(() => Y_SCope5.VisibleRange = new DoubleRange(-10, 10));
                            Y_SCope6.Dispatcher.Invoke(() => Y_SCope6.VisibleRange = new DoubleRange(0, 100));
                            Y_SCope7.Dispatcher.Invoke(() => Y_SCope7.VisibleRange = new DoubleRange(0, 1.2));
                        }
                        
                    }
                    else
                    {
                        //Thread.Sleep(1);
                    }    

                }
                catch
                {
                    //Thread.Sleep(1);
                }
            }
        }

        public void Lidar_Signal()
        {
            // kiểm tra kết nối PC1
            //UInt16 Lat0 = 0;
            //UInt16 Lat1 = 0;
            UInt16 connect = 0;
            UInt16 Counters = 0;

            while (true)
            {
                try
                {
                    Robot_Analyzers.Lidar_Frame = Lidar_Client.Receive(ref Lidar_Clienti);
                    
                    for (int i = 0; i < 360; i++)
                    {
                        Robot_Analyzers._re1[i] = (Robot_Analyzers.Lidar_Frame[i * 2 + 2] * 256 + Robot_Analyzers.Lidar_Frame[i * 2 + 3]) / 1000.0;
                        Robot_Analyzers._im1[i] = i;
                    }

                    // Append data to series. SciChart automatically redraws
                    //dataSeries.Append(data.XData, data.YData);
                    Robot_Analyzers._xyDataSeries.Clear();
                    Robot_Analyzers._xyDataSeries.Append(Robot_Analyzers._im1, Robot_Analyzers._re1);

                    if (connect == 0)
                    {
                        connect = 1;
                        Connect1.Dispatcher.Invoke(() => Connect1.Fill = Brushes.Red);
                    }
                    else if (connect == 1)
                    {
                        connect = 0;
                        Connect1.Dispatcher.Invoke(() => Connect1.Fill = Brushes.Green);
                    }

                }
                catch
                {

                }
            }
        }
        #endregion

        #region process
        void innit_Table()
        {
            DataColumn STT = new DataColumn("STT", typeof(string));
            DataColumn Time = new DataColumn("Thời gian TX1", typeof(string));
            DataColumn Tx = new DataColumn("Dữ liệu kênh TX1", typeof(string));
            DataColumn Time1 = new DataColumn("Thời gian RX1", typeof(string));
            DataColumn Rx = new DataColumn("Dữ liệu kênh RX1", typeof(string));
            DataColumn Time2 = new DataColumn("Thời gian TX2", typeof(string));
            DataColumn Tx2 = new DataColumn("Dữ liệu kênh TX2", typeof(string));
            DataColumn Time3 = new DataColumn("Thời gian RX2", typeof(string));
            DataColumn Rx2 = new DataColumn("Dữ liệu kênh RX2", typeof(string));
            DataColumn Time4 = new DataColumn("Thời gian TX3", typeof(string));
            DataColumn Tx3 = new DataColumn("Dữ liệu kênh TX3", typeof(string));
            DataColumn Time5 = new DataColumn("Thời gian RX3", typeof(string));
            DataColumn Rx3 = new DataColumn("Dữ liệu kênh RX3", typeof(string));

            Robot_Analyzers.KS.Columns.Add(STT); // 0
            Robot_Analyzers.KS.Columns.Add(Time); // 1
            Robot_Analyzers.KS.Columns.Add(Tx); // 2
            Robot_Analyzers.KS.Columns.Add(Time1); // 3
            Robot_Analyzers.KS.Columns.Add(Rx); // 4
            Robot_Analyzers.KS.Columns.Add(Time2); // 5
            Robot_Analyzers.KS.Columns.Add(Tx2); // 6
            Robot_Analyzers.KS.Columns.Add(Time3); // 7
            Robot_Analyzers.KS.Columns.Add(Rx2); // 8
            Robot_Analyzers.KS.Columns.Add(Time4); // 9
            Robot_Analyzers.KS.Columns.Add(Tx3); // 10
            Robot_Analyzers.KS.Columns.Add(Time5); // 11
            Robot_Analyzers.KS.Columns.Add(Rx3); // 12
        }

        private void Playback_time_Tick(object sender, EventArgs e)
        {

        }

        private void PolarChartExampleView_OnLoaded(object sender, RoutedEventArgs e)
        {
            //var dataSeries = new XyDataSeries<double, double>();

            //lineRenderSeries.DataSeries = dataSeries;

            //var data = DataManager.Instance.GetSquirlyWave();

            //for (int i = 0; i < 360; i++)
            //{
            //    _re1[i] = i;
            //    _im1[i] = i;
            //}

            //// Append data to series. SciChart automatically redraws
            ////dataSeries.Append(data.XData, data.YData);

            //dataSeries.Append(_im1, _re1);

            //sciChart.ZoomExtents();
        }

        private void OnXAxisALignmentChanged(object sender, SelectionChangedEventArgs e)
        {
            if (xAxis != null)
                xAxis.AxisAlignment = (AxisAlignment)Enum.Parse(typeof(AxisAlignment), (string)e.AddedItems[0], true);
        }

        private void OnYAxisALignmentChanged(object sender, SelectionChangedEventArgs e)
        {
            if (yAxis != null)
                yAxis.AxisAlignment = (AxisAlignment)Enum.Parse(typeof(AxisAlignment), (string)e.AddedItems[0], true);
        }

        private void Start_View_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            ROBOT_VIEW.Visibility = Visibility.Hidden;
            Lidar_View2.Visibility = Visibility.Hidden;
            Lidar_Viewx.Visibility = Visibility.Visible;
        }

        private void switch_mode2_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            ROBOT_VIEW.Visibility = Visibility.Visible;
            Lidar_Viewx.Visibility = Visibility.Hidden;
            Lidar_View2.Visibility = Visibility.Hidden;
        }

        private void SCREEN_VIEWS_Checked(object sender, RoutedEventArgs e)
        {
            //X_SCope.VisibleRange = new DoubleRange(0, 20000);
        }

        private void SCREEN_VIEWS_Unchecked(object sender, RoutedEventArgs e)
        {
            //X_SCope.VisibleRange = new DoubleRange(-20000, 20000);
        }

        #endregion

        #region Click

        private void BT_CLEAR_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Robot_Analyzers.Clear_Request1 = 1;
        }

        

        private void SCREEN_VIEWS2_Checked(object sender, RoutedEventArgs e)
        {
            Robot_Analyzers.Robot_View_Stop = 0;
            SCREEN_VIEWS22.Content = "RUN";

            if (Robot_Analyzers.Robot_Analyse_Start == 1)
            {
                Reload_Thread();
            }    
        }

        private void SCREEN_VIEWS2_Unchecked(object sender, RoutedEventArgs e)
        {
            Robot_Analyzers.Robot_View_Stop = 1;
            SCREEN_VIEWS22.Content = "STOP";
        }

        private void Clear2_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Robot_Analyzers.Clear_Request2 = 1;
        }

        private void S1_Line1i_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if ((Robot_Analyzers.Robo_Restart == 20) || (Robot_Analyzers.Robot_View_Stop == 1))
            {
                Int16 Point_View2 = Convert.ToInt16(S1_Line1i.X1);

                S1_Line1i.X1 = Point_View2;

                S2_Line1i.X1 = S1_Line1i.X1;
                S3_Line1i.X1 = S1_Line1i.X1;
                S4_Line1i.X1 = S1_Line1i.X1;
                S5_Line1i.X1 = S4_Line1i.X1;

                int Point_View = Point_View2 - 1;
                if (Point_View < 0) Point_View = 0;

                RB_tbx_time.Dispatcher.Invoke(() => RB_tbx_time.Text = string.Concat(Math.Round(((Point_View2 * Robot_Analyzers.Robot_View_Delay) / 1000.0), 2), " s"));

                RB_tb2.Dispatcher.Invoke(() => RB_tb4.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb13.YValues[Point_View]));
                RB_tb4.Dispatcher.Invoke(() => RB_tb4.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb2.YValues[Point_View]));
                RB_tb6.Dispatcher.Invoke(() => RB_tb6.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb3.YValues[Point_View]));
                RB_tb8.Dispatcher.Invoke(() => RB_tb8.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb4.YValues[Point_View]));
                RB_tb10.Dispatcher.Invoke(() => RB_tb10.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb5.YValues[Point_View]));
                RB_tb17.Dispatcher.Invoke(() => RB_tb17.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb6.YValues[Point_View]));
                RB_tb18.Dispatcher.Invoke(() => RB_tb18.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb7.YValues[Point_View]));
                RB_tb20.Dispatcher.Invoke(() => RB_tb20.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb8.YValues[Point_View]));
                RB_tb3.Dispatcher.Invoke(() => RB_tb3.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb9.YValues[Point_View]));
                RB_tb11.Dispatcher.Invoke(() => RB_tb11.Text = string.Concat(Robot_Analyzers._xyDataSeries_IO9.YValues[Point_View]));
                RB_tb12.Dispatcher.Invoke(() => RB_tb12.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb10.YValues[Point_View]));
                RB_tb13.Dispatcher.Invoke(() => RB_tb13.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb11.YValues[Point_View]));
                RB_tb14.Dispatcher.Invoke(() => RB_tb14.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb12.YValues[Point_View]));

            }
        }

        private void S2_Line1i_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if ((Robot_Analyzers.Robo_Restart == 20) || (Robot_Analyzers.Robot_View_Stop == 1))
            {
                Int16 Point_View2 = Convert.ToInt16(S2_Line1i.X1);

                S2_Line1i.X1 = Point_View2;

                S1_Line1i.X1 = S2_Line1i.X1;
                S3_Line1i.X1 = S2_Line1i.X1;
                S4_Line1i.X1 = S2_Line1i.X1;
                S5_Line1i.X1 = S4_Line1i.X1;

                int Point_View = Point_View2 - 1;
                if (Point_View < 0) Point_View = 0;

                RB_tbx_time.Dispatcher.Invoke(() => RB_tbx_time.Text = string.Concat(Math.Round(((Point_View2 * Robot_Analyzers.Robot_View_Delay) / 1000.0), 2), " s"));

                RB_tb2.Dispatcher.Invoke(() => RB_tb2.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb13.YValues[Point_View]));
                RB_tb4.Dispatcher.Invoke(() => RB_tb4.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb2.YValues[Point_View]));
                RB_tb6.Dispatcher.Invoke(() => RB_tb6.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb3.YValues[Point_View]));
                RB_tb8.Dispatcher.Invoke(() => RB_tb8.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb4.YValues[Point_View]));
                RB_tb10.Dispatcher.Invoke(() => RB_tb10.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb5.YValues[Point_View]));
                RB_tb17.Dispatcher.Invoke(() => RB_tb17.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb6.YValues[Point_View]));
                RB_tb18.Dispatcher.Invoke(() => RB_tb18.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb7.YValues[Point_View]));
                RB_tb20.Dispatcher.Invoke(() => RB_tb20.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb8.YValues[Point_View]));
                RB_tb3.Dispatcher.Invoke(() => RB_tb3.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb9.YValues[Point_View]));
                RB_tb11.Dispatcher.Invoke(() => RB_tb11.Text = string.Concat(Robot_Analyzers._xyDataSeries_IO9.YValues[Point_View]));
                RB_tb12.Dispatcher.Invoke(() => RB_tb12.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb10.YValues[Point_View]));
                RB_tb13.Dispatcher.Invoke(() => RB_tb13.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb11.YValues[Point_View]));
                RB_tb14.Dispatcher.Invoke(() => RB_tb14.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb12.YValues[Point_View]));

            }
        }

        private void S3_Line1i_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if ((Robot_Analyzers.Robo_Restart == 20) || (Robot_Analyzers.Robot_View_Stop == 1))
            {
                Int16 Point_View2 = Convert.ToInt16(S3_Line1i.X1);

                S3_Line1i.X1 = Point_View2;

                S2_Line1i.X1 = S3_Line1i.X1;
                S1_Line1i.X1 = S3_Line1i.X1;
                S4_Line1i.X1 = S3_Line1i.X1;
                S5_Line1i.X1 = S4_Line1i.X1;

                int Point_View = Point_View2 - 1;
                if (Point_View < 0) Point_View = 0;

                RB_tbx_time.Dispatcher.Invoke(() => RB_tbx_time.Text = string.Concat(Math.Round(((Point_View2 * Robot_Analyzers.Robot_View_Delay) / 1000.0), 2), " s"));

                RB_tb2.Dispatcher.Invoke(() => RB_tb2.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb13.YValues[Point_View]));
                RB_tb4.Dispatcher.Invoke(() => RB_tb4.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb2.YValues[Point_View]));
                RB_tb6.Dispatcher.Invoke(() => RB_tb6.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb3.YValues[Point_View]));
                RB_tb8.Dispatcher.Invoke(() => RB_tb8.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb4.YValues[Point_View]));
                RB_tb10.Dispatcher.Invoke(() => RB_tb10.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb5.YValues[Point_View]));
                RB_tb17.Dispatcher.Invoke(() => RB_tb17.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb6.YValues[Point_View]));
                RB_tb20.Dispatcher.Invoke(() => RB_tb20.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb8.YValues[Point_View]));
                RB_tb3.Dispatcher.Invoke(() => RB_tb3.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb9.YValues[Point_View]));
                RB_tb11.Dispatcher.Invoke(() => RB_tb11.Text = string.Concat(Robot_Analyzers._xyDataSeries_IO9.YValues[Point_View]));
                RB_tb12.Dispatcher.Invoke(() => RB_tb12.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb10.YValues[Point_View]));
                RB_tb13.Dispatcher.Invoke(() => RB_tb13.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb11.YValues[Point_View]));
                RB_tb14.Dispatcher.Invoke(() => RB_tb14.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb12.YValues[Point_View]));

            }
        }

        private void S4_Line1i_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if ((Robot_Analyzers.Robo_Restart == 20)|| (Robot_Analyzers.Robot_View_Stop == 1))
            {
                Int16 Point_View2 = Convert.ToInt16(S4_Line1i.X1);

                S4_Line1i.X1 = Point_View2;
                S2_Line1i.X1 = S4_Line1i.X1;
                S3_Line1i.X1 = S4_Line1i.X1;
                S1_Line1i.X1 = S4_Line1i.X1;
                S5_Line1i.X1 = S4_Line1i.X1;

                int Point_View = Point_View2 - 1;
                if (Point_View < 0) Point_View = 0;

                RB_tbx_time.Dispatcher.Invoke(() => RB_tbx_time.Text = string.Concat(Math.Round(((Point_View2 * Robot_Analyzers.Robot_View_Delay) / 1000.0), 2), " s"));

                RB_tb2.Dispatcher.Invoke(() => RB_tb2.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb13.YValues[Point_View]));
                RB_tb4.Dispatcher.Invoke(() => RB_tb4.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb2.YValues[Point_View]));
                RB_tb6.Dispatcher.Invoke(() => RB_tb6.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb3.YValues[Point_View]));
                RB_tb8.Dispatcher.Invoke(() => RB_tb8.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb4.YValues[Point_View]));
                RB_tb10.Dispatcher.Invoke(() => RB_tb10.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb5.YValues[Point_View]));
                RB_tb17.Dispatcher.Invoke(() => RB_tb17.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb6.YValues[Point_View]));
                RB_tb18.Dispatcher.Invoke(() => RB_tb18.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb7.YValues[Point_View]));
                RB_tb20.Dispatcher.Invoke(() => RB_tb20.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb8.YValues[Point_View]));
                RB_tb3.Dispatcher.Invoke(() => RB_tb3.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb9.YValues[Point_View]));
                RB_tb11.Dispatcher.Invoke(() => RB_tb11.Text = string.Concat(Robot_Analyzers._xyDataSeries_IO9.YValues[Point_View]));
                RB_tb12.Dispatcher.Invoke(() => RB_tb12.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb10.YValues[Point_View]));
                RB_tb13.Dispatcher.Invoke(() => RB_tb13.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb11.YValues[Point_View]));
                RB_tb14.Dispatcher.Invoke(() => RB_tb14.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb12.YValues[Point_View]));

            }
        }

        private void S5_Linei1_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if ((Robot_Analyzers.Robo_Restart == 20) || (Robot_Analyzers.Robot_View_Stop == 1))
            {
                Int16 Point_View2 = Convert.ToInt16(S5_Line1i.X1);

                S4_Line1i.X1 = Point_View2;
                S2_Line1i.X1 = S5_Line1i.X1;
                S3_Line1i.X1 = S5_Line1i.X1;
                S1_Line1i.X1 = S5_Line1i.X1;
                S4_Line1i.X1 = S5_Line1i.X1;

                int Point_View = Point_View2 - 1;
                if (Point_View < 0) Point_View = 0;

                RB_tbx_time.Dispatcher.Invoke(() => RB_tbx_time.Text = string.Concat(Math.Round(((Point_View2 * Robot_Analyzers.Robot_View_Delay) / 1000.0), 2), " s"));

                RB_tb2.Dispatcher.Invoke(() => RB_tb2.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb13.YValues[Point_View]));
                RB_tb4.Dispatcher.Invoke(() => RB_tb4.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb2.YValues[Point_View]));
                RB_tb6.Dispatcher.Invoke(() => RB_tb6.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb3.YValues[Point_View]));
                RB_tb8.Dispatcher.Invoke(() => RB_tb8.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb4.YValues[Point_View]));
                RB_tb10.Dispatcher.Invoke(() => RB_tb10.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb5.YValues[Point_View]));
                RB_tb17.Dispatcher.Invoke(() => RB_tb17.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb6.YValues[Point_View]));
                RB_tb18.Dispatcher.Invoke(() => RB_tb18.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb7.YValues[Point_View]));
                RB_tb20.Dispatcher.Invoke(() => RB_tb20.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb8.YValues[Point_View]));
                RB_tb3.Dispatcher.Invoke(() => RB_tb3.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb9.YValues[Point_View]));
                RB_tb11.Dispatcher.Invoke(() => RB_tb11.Text = string.Concat(Robot_Analyzers._xyDataSeries_IO9.YValues[Point_View]));
                RB_tb12.Dispatcher.Invoke(() => RB_tb12.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb10.YValues[Point_View]));
                RB_tb13.Dispatcher.Invoke(() => RB_tb13.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb11.YValues[Point_View]));
                RB_tb14.Dispatcher.Invoke(() => RB_tb14.Text = string.Concat(Robot_Analyzers._xyDataSeries_rb12.YValues[Point_View]));

            }
        }

        private void ClearX1_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            byte[] Data_Send = new byte[11];

            Data_Send[0] = (byte)'R';
            Data_Send[1] = (byte)'E';
            Data_Send[2] = (byte)'S';
            Data_Send[3] = (byte)'E';
            Data_Send[4] = (byte)'T';
            Data_Send[5] = (byte)'_';
            Data_Send[6] = (byte)'A';
            Data_Send[7] = (byte)'N';
            Data_Send[8] = (byte)'G';
            Data_Send[9] = (byte)'L';
            Data_Send[10] = (byte)'E';

            Compass_Client.Send(Data_Send, 11, Compass_Cliento);
            Robot_Analyzers.Clear_Request2 = 1;
        }

        private void ClearX2_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            byte[] Data_Send = new byte[11];

            Data_Send[0] = (byte)'C';
            Data_Send[1] = (byte)'A';
            Data_Send[2] = (byte)'L';
            Data_Send[3] = (byte)'I';
            Data_Send[4] = (byte)'B';

            Compass_Client.Send(Data_Send, 5, Compass_Cliento);
            Robot_Analyzers.Clear_Request2 = 1;
        }

        private void S9_Line1i_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if ((Robot_Analyzers.Robo_Restart2 == 20) || (Robot_Analyzers.Robot_View_Stop2 == 1))
            {
                Int16 Point_View2 = Convert.ToInt16(S9_Line1i.X1);

                S8_Line1i.X1 = Point_View2;

                int Point_View = Point_View2 - 1;
                if (Point_View < 0) Point_View = 0;

                RAM_D1.Dispatcher.Invoke(() => RAM_D1.Text = string.Concat(Robot_Analyzers._xyDataSeries_R1.YValues[Point_View]));
                RAM_D2.Dispatcher.Invoke(() => RAM_D2.Text = string.Concat(Robot_Analyzers._xyDataSeries_R2.YValues[Point_View]));
                RAM_D3.Dispatcher.Invoke(() => RAM_D3.Text = string.Concat(Robot_Analyzers._xyDataSeries_R3.YValues[Point_View]));
                RAM_D4.Dispatcher.Invoke(() => RAM_D4.Text = string.Concat(Robot_Analyzers._xyDataSeries_R4.YValues[Point_View]));

                Result_D1.Dispatcher.Invoke(() => Result_D1.Text = string.Concat(Robot_Analyzers._xyDataSeries_A1.YValues[Point_View]));
                Result_D2.Dispatcher.Invoke(() => Result_D2.Text = string.Concat(Robot_Analyzers._xyDataSeries_A2.YValues[Point_View]));
                Result_D3.Dispatcher.Invoke(() => Result_D3.Text = string.Concat(Robot_Analyzers._xyDataSeries_A3.YValues[Point_View]));
                Result_D4.Dispatcher.Invoke(() => Result_D4.Text = string.Concat(Robot_Analyzers._xyDataSeries_A4.YValues[Point_View]));
                Result_D5.Dispatcher.Invoke(() => Result_D5.Text = string.Concat(Robot_Analyzers._xyDataSeries_A5.YValues[Point_View]));

            }
        }

        private void S8_Line1i_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if ((Robot_Analyzers.Robo_Restart2 == 20) || (Robot_Analyzers.Robot_View_Stop2 == 1))
            {
                Int16 Point_View2 = Convert.ToInt16(S8_Line1i.X1);

                S9_Line1i.X1 = Point_View2;

                int Point_View = Point_View2 - 1;
                if (Point_View < 0) Point_View = 0;

                RAM_D1.Dispatcher.Invoke(() => RAM_D1.Text = string.Concat(Robot_Analyzers._xyDataSeries_R1.YValues[Point_View]));
                RAM_D2.Dispatcher.Invoke(() => RAM_D2.Text = string.Concat(Robot_Analyzers._xyDataSeries_R2.YValues[Point_View]));
                RAM_D3.Dispatcher.Invoke(() => RAM_D3.Text = string.Concat(Robot_Analyzers._xyDataSeries_R3.YValues[Point_View]));
                RAM_D4.Dispatcher.Invoke(() => RAM_D4.Text = string.Concat(Robot_Analyzers._xyDataSeries_R4.YValues[Point_View]));

                Result_D1.Dispatcher.Invoke(() => Result_D1.Text = string.Concat(Robot_Analyzers._xyDataSeries_A1.YValues[Point_View]));
                Result_D2.Dispatcher.Invoke(() => Result_D2.Text = string.Concat(Robot_Analyzers._xyDataSeries_A2.YValues[Point_View]));
                Result_D3.Dispatcher.Invoke(() => Result_D3.Text = string.Concat(Robot_Analyzers._xyDataSeries_A3.YValues[Point_View]));
                Result_D4.Dispatcher.Invoke(() => Result_D4.Text = string.Concat(Robot_Analyzers._xyDataSeries_A4.YValues[Point_View]));
                Result_D5.Dispatcher.Invoke(() => Result_D5.Text = string.Concat(Robot_Analyzers._xyDataSeries_A5.YValues[Point_View]));

            }
        }

        private void SCREEN_VIEWS22_Checked(object sender, RoutedEventArgs e)
        {
            Robot_Analyzers.Robot_View_Stop2 = 0;
        }

        private void SCREEN_VIEWS22_Unchecked(object sender, RoutedEventArgs e)
        {
            Robot_Analyzers.Robot_View_Stop2 = 1;
        }

        private void SCREEN_VIEWS23_Checked(object sender, RoutedEventArgs e)
        {
            Robot_Analyzers.Robot_View_SYNC = 1;
            Sensor_Data_A5.Stroke = Color.FromArgb(255, 238, 130, 238);
        }

        private void SCREEN_VIEWS23_Unchecked(object sender, RoutedEventArgs e)
        {
            Robot_Analyzers.Robot_View_SYNC = 0;
            Sensor_Data_A5.Stroke = Color.FromArgb(0, 238, 130, 238);
        }

        private void Start_View_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            ROBOT_VIEW.Visibility = Visibility.Hidden;
            Lidar_View2.Visibility = Visibility.Hidden;
            Lidar_Viewx.Visibility = Visibility.Visible;
        }

        private void switch_mode2_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            ROBOT_VIEW.Visibility = Visibility.Visible;
            Lidar_Viewx.Visibility = Visibility.Hidden;
            Lidar_View2.Visibility = Visibility.Hidden;
        }

        private void Clear2_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Robot_Analyzers.Clear_Request2 = 1;
        }

        private void ClearX1_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            byte[] Data_Send = new byte[11];

            Data_Send[0] = (byte)'R';
            Data_Send[1] = (byte)'E';
            Data_Send[2] = (byte)'S';
            Data_Send[3] = (byte)'E';
            Data_Send[4] = (byte)'T';
            Data_Send[5] = (byte)'_';
            Data_Send[6] = (byte)'A';
            Data_Send[7] = (byte)'N';
            Data_Send[8] = (byte)'G';
            Data_Send[9] = (byte)'L';
            Data_Send[10] = (byte)'E';

            Compass_Client.Send(Data_Send, 11, Compass_Cliento);
            Robot_Analyzers.Clear_Request2 = 1;
        }

        private void ClearX2_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            byte[] Data_Send = new byte[11];

            Data_Send[0] = (byte)'C';
            Data_Send[1] = (byte)'A';
            Data_Send[2] = (byte)'L';
            Data_Send[3] = (byte)'I';
            Data_Send[4] = (byte)'B';

            Compass_Client.Send(Data_Send, 5, Compass_Cliento);
            Robot_Analyzers.Clear_Request2 = 1;
        }

        private void Start_View2_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            ROBOT_VIEW.Visibility = Visibility.Hidden;
            Lidar_View2.Visibility = Visibility.Visible;
            Lidar_Viewx.Visibility = Visibility.Hidden;
        }

        private void Start_View2_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            ROBOT_VIEW.Visibility = Visibility.Hidden;
            Lidar_View2.Visibility = Visibility.Visible;
            Lidar_Viewx.Visibility = Visibility.Hidden;
        }

        private void Start_View3_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Main_View.Visibility = Visibility.Visible;
        }

        private void Start_View3_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Main_View.Visibility = Visibility.Visible;
        }

        private void Start_View_Exit_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Main_View.Visibility = Visibility.Hidden;
            Setup_eeprom.Visibility = Visibility.Hidden;
        }

        private void Start_View_Exit_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Main_View.Visibility = Visibility.Hidden;
            Setup_eeprom.Visibility = Visibility.Hidden;
        }

        private void Start_View_cLEAR_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Robot_Analyzers.Robot_Clear = 1;
        }

        private void Start_View_cLEAR_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Robot_Analyzers.Robot_Clear = 1;
        }

        private void comboBox_u1_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            Robot_Analyzers.Robot_View1 = comboBox_u1.SelectedIndex; 
            Robot_Analyzers.Robot_Clear = 1;
        }

        private void comboBox_u2_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            Robot_Analyzers.Robot_View2 = comboBox_u2.SelectedIndex; 
            Robot_Analyzers.Robot_Clear = 1;
        }

        private void comboBox_u22_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            Robot_Analyzers.Robot_View4 = comboBox_u22.SelectedIndex;
            Robot_Analyzers.Robot_Clear = 1;
        }

        private void comboBox_u11_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            Robot_Analyzers.Robot_View3 = comboBox_u11.SelectedIndex;
            Robot_Analyzers.Robot_Clear = 1;
        }

        private void comboBox_Scale1_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (comboBox_Scale1.SelectedIndex == 0) Robot_Analyzers.Scale1 = 5;
            else if (comboBox_Scale1.SelectedIndex == 1) Robot_Analyzers.Scale1 = 10;
            else if (comboBox_Scale1.SelectedIndex == 2) Robot_Analyzers.Scale1 = 50;
            else if (comboBox_Scale1.SelectedIndex == 3) Robot_Analyzers.Scale1 = 100;
            else if (comboBox_Scale1.SelectedIndex == 4) Robot_Analyzers.Scale1 = 250;
            else if (comboBox_Scale1.SelectedIndex == 5) Robot_Analyzers.Scale1 = 500;
            else if (comboBox_Scale1.SelectedIndex == 6) Robot_Analyzers.Scale1 = 1000;
            else if (comboBox_Scale1.SelectedIndex == 7) Robot_Analyzers.Scale1 = 2000;
            else if (comboBox_Scale1.SelectedIndex == 8) Robot_Analyzers.Scale1 = 4000;
            else if (comboBox_Scale1.SelectedIndex == 9) Robot_Analyzers.Scale1 = 8000;
            else if (comboBox_Scale1.SelectedIndex == 10) Robot_Analyzers.Scale1 = 16000;
            else if (comboBox_Scale1.SelectedIndex == 11) Robot_Analyzers.Scale1 = 32000;
        }

        private void comboBox_Scale2_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (comboBox_Scale2.SelectedIndex == 0) Robot_Analyzers.Scale2 = 5;
            else if (comboBox_Scale2.SelectedIndex == 1) Robot_Analyzers.Scale2 = 10;
            else if (comboBox_Scale2.SelectedIndex == 2) Robot_Analyzers.Scale2 = 50;
            else if (comboBox_Scale2.SelectedIndex == 3) Robot_Analyzers.Scale2 = 100;
            else if (comboBox_Scale2.SelectedIndex == 4) Robot_Analyzers.Scale2 = 250;
            else if (comboBox_Scale2.SelectedIndex == 5) Robot_Analyzers.Scale2 = 500;
            else if (comboBox_Scale2.SelectedIndex == 6) Robot_Analyzers.Scale2 = 1000;
            else if (comboBox_Scale2.SelectedIndex == 7) Robot_Analyzers.Scale2 = 2000;
            else if (comboBox_Scale2.SelectedIndex == 8) Robot_Analyzers.Scale2 = 4000;
            else if (comboBox_Scale2.SelectedIndex == 9) Robot_Analyzers.Scale2 = 8000;
            else if (comboBox_Scale2.SelectedIndex == 10) Robot_Analyzers.Scale2 = 16000;
            else if (comboBox_Scale2.SelectedIndex == 11) Robot_Analyzers.Scale2 = 32000;

        }

        private void comboBox_Scale3_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (comboBox_Scale3.SelectedIndex == 0) Robot_Analyzers.Scale3 = 5;
            else if (comboBox_Scale3.SelectedIndex == 1) Robot_Analyzers.Scale3 = 10;
            else if (comboBox_Scale3.SelectedIndex == 2) Robot_Analyzers.Scale3 = 50;
            else if (comboBox_Scale3.SelectedIndex == 3) Robot_Analyzers.Scale3 = 100;
            else if (comboBox_Scale3.SelectedIndex == 4) Robot_Analyzers.Scale3 = 250;
            else if (comboBox_Scale3.SelectedIndex == 5) Robot_Analyzers.Scale3 = 500;
            else if (comboBox_Scale3.SelectedIndex == 6) Robot_Analyzers.Scale3 = 1000;
            else if (comboBox_Scale3.SelectedIndex == 7) Robot_Analyzers.Scale3 = 2000;
            else if (comboBox_Scale3.SelectedIndex == 8) Robot_Analyzers.Scale3 = 4000;
            else if (comboBox_Scale3.SelectedIndex == 9) Robot_Analyzers.Scale3 = 8000;
            else if (comboBox_Scale3.SelectedIndex == 10) Robot_Analyzers.Scale3 = 16000;
            else if (comboBox_Scale3.SelectedIndex == 11) Robot_Analyzers.Scale3 = 32000;

        }

        #endregion

        #region Eeprom

        private void Start_View_EEPROM_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Setup_eeprom.Visibility = Visibility.Visible;
            Robot_Analyzers.EEprom_Save = 1;
            Robot_Analyzers.EEprom_Mode = 1;
        }

        private void Start_View_EEPROM_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Setup_eeprom.Visibility = Visibility.Visible;
            Robot_Analyzers.EEprom_Save = 1;
            Robot_Analyzers.EEprom_Mode = 1;
        }

        private void bt_eeprom_Exit_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Setup_eeprom.Visibility = Visibility.Hidden;
            Robot_Analyzers.EEprom_Save = 0;
            Robot_Analyzers.EEprom_Mode = 0;
        }

        private void bt_eeprom_Exit_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Setup_eeprom.Visibility = Visibility.Hidden;
            Robot_Analyzers.EEprom_Save = 0;
            Robot_Analyzers.EEprom_Mode = 0;
        }

        private void bt_eeprom_readfile_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {

        }

        private void bt_eeprom_readfile_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {

        }

        private void bt_eeprom_copy_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Eeprom_Load_Data();
        }

        private void bt_eeprom_copy_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Eeprom_Load_Data();
        }

        public void Eeprom_Save_Data()
        {
            Robot_Analyzers.Robot_EEprom[0] = Convert.ToUInt16(RB_SV1.Text);
            Robot_Analyzers.Robot_EEprom[1] = Convert.ToUInt16(RB_SV2.Text);
            Robot_Analyzers.Robot_EEprom[2] = Convert.ToUInt16(RB_SV3.Text);
            Robot_Analyzers.Robot_EEprom[3] = Convert.ToUInt16(RB_SV4.Text);
            Robot_Analyzers.Robot_EEprom[4] = Convert.ToUInt16(RB_SV5.Text);
            Robot_Analyzers.Robot_EEprom[5] = Convert.ToUInt16(RB_SV6.Text);
            Robot_Analyzers.Robot_EEprom[6] = Convert.ToUInt16(RB_SV7.Text);
            Robot_Analyzers.Robot_EEprom[7] = Convert.ToUInt16(RB_SV8.Text);
            Robot_Analyzers.Robot_EEprom[8] = Convert.ToUInt16(RB_SV9.Text);
            Robot_Analyzers.Robot_EEprom[9] = Convert.ToUInt16(RB_SV10.Text);
            Robot_Analyzers.Robot_EEprom[10] = Convert.ToUInt16(RB_SV11.Text);
            Robot_Analyzers.Robot_EEprom[11] = Convert.ToUInt16(RB_SV12.Text);
            Robot_Analyzers.Robot_EEprom[12] = Convert.ToUInt16(RB_SV13.Text);
            Robot_Analyzers.Robot_EEprom[13] = Convert.ToUInt16(RB_SV14.Text);
            Robot_Analyzers.Robot_EEprom[14] = Convert.ToUInt16(RB_SV15.Text);
            Robot_Analyzers.Robot_EEprom[15] = Convert.ToUInt16(RB_SV16.Text);
            Robot_Analyzers.Robot_EEprom[16] = Convert.ToUInt16(RB_SV17.Text);
            Robot_Analyzers.Robot_EEprom[17] = Convert.ToUInt16(RB_SV18.Text);
            Robot_Analyzers.Robot_EEprom[18] = Convert.ToUInt16(RB_SV19.Text);
            Robot_Analyzers.Robot_EEprom[19] = Convert.ToUInt16(RB_SV20.Text);
            Robot_Analyzers.EEprom_Save = 2;
            Robot_Analyzers.EEprom_Read_Point = 0;

        }

        public void Eeprom_Load_Data()
        {
            Robot_Analyzers.Robot_EEprom[0] = Convert.ToUInt16(RB_SVF1.Text);
            Robot_Analyzers.Robot_EEprom[1] = Convert.ToUInt16(RB_SVF2.Text);
            Robot_Analyzers.Robot_EEprom[2] = Convert.ToUInt16(RB_SVF3.Text);
            Robot_Analyzers.Robot_EEprom[3] = Convert.ToUInt16(RB_SVF4.Text);
            Robot_Analyzers.Robot_EEprom[4] = Convert.ToUInt16(RB_SVF5.Text);
            Robot_Analyzers.Robot_EEprom[5] = Convert.ToUInt16(RB_SVF6.Text);
            Robot_Analyzers.Robot_EEprom[6] = Convert.ToUInt16(RB_SVF7.Text);
            Robot_Analyzers.Robot_EEprom[7] = Convert.ToUInt16(RB_SVF8.Text);
            Robot_Analyzers.Robot_EEprom[8] = Convert.ToUInt16(RB_SVF9.Text);
            Robot_Analyzers.Robot_EEprom[9] = Convert.ToUInt16(RB_SVF10.Text);
            Robot_Analyzers.Robot_EEprom[10] = Convert.ToUInt16(RB_SVF11.Text);
            Robot_Analyzers.Robot_EEprom[11] = Convert.ToUInt16(RB_SVF12.Text);
            Robot_Analyzers.Robot_EEprom[12] = Convert.ToUInt16(RB_SVF13.Text);
            Robot_Analyzers.Robot_EEprom[13] = Convert.ToUInt16(RB_SVF14.Text);
            Robot_Analyzers.Robot_EEprom[14] = Convert.ToUInt16(RB_SVF15.Text);
            Robot_Analyzers.Robot_EEprom[15] = Convert.ToUInt16(RB_SVF16.Text);
            Robot_Analyzers.Robot_EEprom[16] = Convert.ToUInt16(RB_SVF17.Text);
            Robot_Analyzers.Robot_EEprom[17] = Convert.ToUInt16(RB_SVF18.Text);
            Robot_Analyzers.Robot_EEprom[18] = Convert.ToUInt16(RB_SVF19.Text);
            Robot_Analyzers.Robot_EEprom[19] = Convert.ToUInt16(RB_SVF20.Text);
            Robot_Analyzers.EEprom_Load = 1;

            RB_SV1.Text = string.Concat(Robot_Analyzers.Robot_EEprom[0]);// = Convert.ToUInt16(RB_SV1.Text);
            RB_SV2.Text = string.Concat(Robot_Analyzers.Robot_EEprom[1]);//  = Convert.ToUInt16(RB_SV2.Text);
            RB_SV3.Text = string.Concat(Robot_Analyzers.Robot_EEprom[2]);//  = Convert.ToUInt16(RB_SV3.Text);
            RB_SV4.Text = string.Concat(Robot_Analyzers.Robot_EEprom[3]);//  = Convert.ToUInt16(RB_SV4.Text);
            RB_SV5.Text = string.Concat(Robot_Analyzers.Robot_EEprom[4]);//  = Convert.ToUInt16(RB_SV5.Text);
            RB_SV6.Text = string.Concat(Robot_Analyzers.Robot_EEprom[5]);//  = Convert.ToUInt16(RB_SV6.Text);
            RB_SV7.Text = string.Concat(Robot_Analyzers.Robot_EEprom[6]);//  = Convert.ToUInt16(RB_SV7.Text);
            RB_SV8.Text = string.Concat(Robot_Analyzers.Robot_EEprom[7]);//  = Convert.ToUInt16(RB_SV8.Text);
            RB_SV9.Text = string.Concat(Robot_Analyzers.Robot_EEprom[8]);//  = Convert.ToUInt16(RB_SV9.Text);
            RB_SV10.Text = string.Concat(Robot_Analyzers.Robot_EEprom[9]);//  = Convert.ToUInt16(RB_SV10.Text);
            RB_SV11.Text = string.Concat(Robot_Analyzers.Robot_EEprom[10]);//  = Convert.ToUInt16(RB_SV11.Text);
            RB_SV12.Text = string.Concat(Robot_Analyzers.Robot_EEprom[11]);//  = Convert.ToUInt16(RB_SV12.Text);
            RB_SV13.Text = string.Concat(Robot_Analyzers.Robot_EEprom[12]);//  = Convert.ToUInt16(RB_SV13.Text);
            RB_SV14.Text = string.Concat(Robot_Analyzers.Robot_EEprom[13]);//  = Convert.ToUInt16(RB_SV14.Text);
            RB_SV15.Text = string.Concat(Robot_Analyzers.Robot_EEprom[14]);//  = Convert.ToUInt16(RB_SV15.Text);
            RB_SV16.Text = string.Concat(Robot_Analyzers.Robot_EEprom[15]);//  = Convert.ToUInt16(RB_SV16.Text);
            RB_SV17.Text = string.Concat(Robot_Analyzers.Robot_EEprom[16]);//  = Convert.ToUInt16(RB_SV17.Text);
            RB_SV18.Text = string.Concat(Robot_Analyzers.Robot_EEprom[17]);//  = Convert.ToUInt16(RB_SV18.Text);
            RB_SV19.Text = string.Concat(Robot_Analyzers.Robot_EEprom[18]);//  = Convert.ToUInt16(RB_SV19.Text);
            RB_SV20.Text = string.Concat(Robot_Analyzers.Robot_EEprom[19]);//  = Convert.ToUInt16(RB_SV20.Text);


        }

        private void bt_eeprom_save_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Eeprom_Save_Data();
        }

        private void bt_eeprom_save_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Eeprom_Save_Data();
        }

        private void RB_SVF1_TextChanged(object sender, TextChangedEventArgs e)
        {

        }
    }
    #endregion
}
