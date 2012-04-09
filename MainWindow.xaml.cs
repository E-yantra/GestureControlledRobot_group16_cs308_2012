/////////////////////////////////////////////////////////////////////////
//   Project : Gesture Controlled Robot
//   File  : MainWindow.xaml.cs
//   Class : MainWindow
//   Description :
//              1. Processes Depth-data and Sekeltal-data read by 
//                 Kinect sensor
//              2. Updates Main window of the application
//              3. Recognizes gestures
//              4. Sends command to bot using Zigbee module through 
//                 serial port
//                
//   Date of Creation :     NA
//   Created by :           Microsoft
//   Date of Modification : September-November, 2011
//   Created by :           CS-684 Project Group-16
//   NOTE       :           For embedded comment look for "TEAM 16_2011"
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
//
// This module contains code to do Kinect NUI initialization and
// processing and also to display NUI streams on screen.
//
// Copyright � Microsoft Corporation.  All rights reserved.  
// This code is licensed under the terms of the 
// Microsoft Kinect for Windows SDK (Beta) from Microsoft Research 
// License Agreement: http://research.microsoft.com/KinectSDK-ToU
//
/////////////////////////////////////////////////////////////////////////
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Research.Kinect.Nui;
using System.IO.Ports;


namespace SkeletalViewer
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Constructors
        public MainWindow()
        {
            InitializeComponent();
            // TEAM 16_2011
            // Initialize the serial port and open it for communication.
            // Check the device manager setting to get the correct COM
            // port number for your Zigbee module
            this.serialPort = new SerialPort("COM3", 9600, Parity.None, 8, StopBits.One);
            serialPort.Handshake = Handshake.None;
            try
            {
                this.serialPort.Open();
            }
            catch (Exception e)
            {
                MessageBox.Show("Error opening serial port for communication.", e.Message, MessageBoxButton.OK, MessageBoxImage.Error);
            }

            this.state = false;
            this.clapStarted = false;
        }
        #endregion

        #region Variables for initialization of Kinect
        Runtime nui;

        int lastFrames = 0;
        int framediff = 30;
        int totalFrames = 10;
        double initangle = 0;
        double curangle = 0;
        int aflag = 0;
        int lim = 10;
        int send;
        int curx = 0;
        DateTime lastTime = DateTime.MaxValue;
        double rprevz;
        char s = '0';

        // We want to control how depth data gets converted into false-color data
        // for more intuitive visualization, so we keep 32-bit color frame buffer versions of
        // these, to be updated whenever we receive and process a 16-bit frame.
        const int RED_IDX = 2;
        const int GREEN_IDX = 1;
        const int BLUE_IDX = 0;
        byte[] depthFrame32 = new byte[320 * 240 * 4];

        Dictionary<JointID, Brush> jointColors = new Dictionary<JointID, Brush>() { 
            {JointID.HipCenter, new SolidColorBrush(Color.FromRgb(169, 176, 155))},
            {JointID.Spine, new SolidColorBrush(Color.FromRgb(169, 176, 155))},
            {JointID.ShoulderCenter, new SolidColorBrush(Color.FromRgb(168, 230, 29))},
            {JointID.Head, new SolidColorBrush(Color.FromRgb(200, 0,   0))},
            {JointID.ShoulderLeft, new SolidColorBrush(Color.FromRgb(79,  84,  33))},
            {JointID.ElbowLeft, new SolidColorBrush(Color.FromRgb(84,  33,  42))},
            {JointID.WristLeft, new SolidColorBrush(Color.FromRgb(255, 126, 0))},
            {JointID.HandLeft, new SolidColorBrush(Color.FromRgb(215,  86, 0))},
            {JointID.ShoulderRight, new SolidColorBrush(Color.FromRgb(33,  79,  84))},
            {JointID.ElbowRight, new SolidColorBrush(Color.FromRgb(33,  33,  84))},
            {JointID.WristRight, new SolidColorBrush(Color.FromRgb(77,  109, 243))},
            {JointID.HandRight, new SolidColorBrush(Color.FromRgb(37,   69, 243))},
            {JointID.HipLeft, new SolidColorBrush(Color.FromRgb(77,  109, 243))},
            {JointID.KneeLeft, new SolidColorBrush(Color.FromRgb(69,  33,  84))},
            {JointID.AnkleLeft, new SolidColorBrush(Color.FromRgb(229, 170, 122))},
            {JointID.FootLeft, new SolidColorBrush(Color.FromRgb(255, 126, 0))},
            {JointID.HipRight, new SolidColorBrush(Color.FromRgb(181, 165, 213))},
            {JointID.KneeRight, new SolidColorBrush(Color.FromRgb(71, 222,  76))},
            {JointID.AnkleRight, new SolidColorBrush(Color.FromRgb(245, 228, 156))},
            {JointID.FootRight, new SolidColorBrush(Color.FromRgb(77,  109, 243))}
        };
        #endregion

        #region Our Variables
        // TEAM 16_2011
        /// <summary>
        /// Variable to track if the robot should be moving or should be in a stop state.
        /// </summary>
        private bool state;

        // TEAM 16_2011
        /// <summary>
        /// Flag to detect if the user is in the middle of a clap.
        /// </summary>
        private bool clapStarted;

        public struct str
        {
            public double x, y, z;


        };

        // TEAM 16_2011
        /// <summary>
        /// Flag to detect if the user is in the middle of a wave motion.
        /// </summary>
        SerialPort serialPort;
        private bool leftWaveStarted;
        private bool rightWaveStarted;
        public str r_wrist;
        public str r_elbow;
        public str r_shldr;
        public str r_hand;
        public str l_wrist;
        public str l_elbow;
        public str l_shldr;
        public str l_hand;
        public str shldr_center;
        public str hip_center;
        public str head;
        public str glob1;
        public double zin1;
        public double zin2;
        public double zvar;


        int p = 0;
        int fck = 10;
        int fcktemp = 0;
        double thresh = 40;
        public int rm_ctrl = 0;
        public int rm_start = 0;
        public int rm_fwd = 0;
        public int rm_bck = 0;
        public DateTime rm_start_t;
        public int r_rot = 0;
        public int r_is = 0;
        public int r_ds = 0;

        public int mode = 0;



        // TEAM 16_2011
        // Variables for serial port communication.
        //SerialPort serialPort;

        // TEAM 16_2011
        // Variables to track the position of various joints and co-ordinates.
        double r_wrist_y = 0;
        double l_wrist_y = 0;
        double r_wrist_x = 0;
        double l_wrist_x = 0;
        double r_wrist_z = 0;
        double l_wrist_z = 0;

        double head_y = 0;
        double head_z = 0;

        private double centreX;
        private double centreY;

        private double r_shoulder_x = 0;
        private double r_shoulder_y = 0;
        private double l_shoulder_x = 0;
        private double l_shoulder_y = 0;

        private double leftWaveStarty;
        private double leftWaveStartx;

        private double rightWaveStartx;
        private double rightWaveStarty;
        #endregion

        private void Window_Loaded(object sender, EventArgs e)
        {
            // TEAM 16_2011
            // Initialization
            nui = Runtime.Kinects[0];

            try
            {
                nui.Initialize(RuntimeOptions.UseDepthAndPlayerIndex | RuntimeOptions.UseSkeletalTracking | RuntimeOptions.UseColor);
            }
            catch (InvalidOperationException)
            {
                System.Windows.MessageBox.Show("Runtime initialization failed. Please make sure Kinect device is plugged in.");
                return;
            }

            try
            {
                nui.VideoStream.Open(ImageStreamType.Video, 2, ImageResolution.Resolution640x480, ImageType.Color);
                nui.DepthStream.Open(ImageStreamType.Depth, 2, ImageResolution.Resolution320x240, ImageType.DepthAndPlayerIndex);

            }
            catch (InvalidOperationException)
            {
                System.Windows.MessageBox.Show("Failed to open stream. Please make sure to specify a supported image type and resolution.");
                return;
            }

            lastTime = DateTime.Now;

            // TEAM 16_2011
            // Setting function 'nui_DepthFrameReady' as event handler of depth frame. 
            // As a depth frames comes from Kinect to be processed, this function will be called.
            // Similarly setting function 'nui_SkeletonFrameReady' and
            // 'nui_ColorFrameReady' as event handlers.
            nui.DepthFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_DepthFrameReady);
            nui.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
            nui.VideoFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_ColorFrameReady);
        }

        // Converts a 16-bit grayscale depth frame which includes player indexes into a 32-bit frame
        // that displays different players in different colors
        byte[] convertDepthFrame(byte[] depthFrame16)
        {
            for (int i16 = 0, i32 = 0; i16 < depthFrame16.Length && i32 < depthFrame32.Length; i16 += 2, i32 += 4)
            {
                int player = depthFrame16[i16] & 0x07;
                int realDepth = (depthFrame16[i16 + 1] << 5) | (depthFrame16[i16] >> 3);
                // transform 13-bit depth information into an 8-bit intensity appropriate
                // for display (we disregard information in most significant bit)
                byte intensity = (byte)(255 - (255 * realDepth / 0x0fff));

                depthFrame32[i32 + RED_IDX] = 0;
                depthFrame32[i32 + GREEN_IDX] = 0;
                depthFrame32[i32 + BLUE_IDX] = 0;

                // choose different display colors based on player
                switch (player)
                {
                    case 0:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity / 2);
                        break;
                    case 1:
                        depthFrame32[i32 + RED_IDX] = intensity;
                        break;
                    case 2:
                        depthFrame32[i32 + GREEN_IDX] = intensity;
                        break;
                    case 3:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity / 4);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity);
                        break;
                    case 4:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity / 4);
                        break;
                    case 5:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity / 4);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity);
                        break;
                    case 6:
                        depthFrame32[i32 + RED_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(intensity / 2);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(intensity);
                        break;
                    case 7:
                        depthFrame32[i32 + RED_IDX] = (byte)(255 - intensity);
                        depthFrame32[i32 + GREEN_IDX] = (byte)(255 - intensity);
                        depthFrame32[i32 + BLUE_IDX] = (byte)(255 - intensity);
                        break;
                }
            }
            return depthFrame32;
        }

        void nui_DepthFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            PlanarImage Image = e.ImageFrame.Image;
            byte[] convertedDepthFrame = convertDepthFrame(Image.Bits);

            depth.Source = BitmapSource.Create(
                Image.Width, Image.Height, 96, 96, PixelFormats.Bgr32, null, convertedDepthFrame, Image.Width * 4);

            //++totalFrames;

            DateTime cur = DateTime.Now;
            /* if (cur.Subtract(lastTime) > TimeSpan.FromSeconds(1))
             {
                 int frameDiff = totalFrames - lastFrames;
                 lastFrames = totalFrames;
                 lastTime = cur;
                 // frameRate.Text = frameDiff.ToString() + " fps by team 16";
             }
             * /
             /*
             */
            str r_we = diff(r_wrist, r_elbow);
            str r_es = diff(r_elbow, r_shldr);
            str r_se = diff(r_shldr, r_elbow);
            str mid = diff(shldr_center, hip_center);
            str pln = diff(r_shldr, l_shldr);
            str r_wc = diff(shldr_center, r_wrist);
            str r_hw = diff(r_hand, r_wrist);
            str sl_sr = diff(l_shldr, r_shldr);

            str l_we = diff(l_wrist, l_elbow);
            str l_es = diff(l_elbow, l_shldr);
            str l_se = diff(l_shldr, l_elbow);
            str l_wc = diff(shldr_center, l_wrist);
            str l_hw = diff(l_hand, l_wrist);



            str r_pt;
            r_pt = r_wrist;
            r_pt.x = r_shldr.x;
            str l_pt;
            l_pt = l_wrist;
            l_pt.x = l_shldr.x;
            str r_req = diff(r_shldr, r_pt);
            str l_req = diff(l_shldr, l_pt);
            zin1 = angle(shldr_center, sl_sr);
            zin2 = angle(shldr_center, r_se);

            //double zvar = angle(shldr_center, r_es);

            double rx1 = angle(r_we, r_es);
            double rx2 = angle(r_es, mid);

            double rx3 = angle(r_se, r_we);
            double rx4 = angle(r_wc, pln);
            double rx5 = angle(r_wc, mid);
            double rx6 = angle(sl_sr, r_se);
            double rx8 = angle(r_we, mid);
            double rx7 = angle(r_we, pln);
            double rx9 = angle(r_es, pln);
            double rx10 = angle(r_req, mid);




            double lx1 = angle(l_we, l_es);
            double lx2 = angle(l_es, mid);

            double lx3 = angle(l_se, l_we);
            double lx4 = angle(l_wc, pln);
            double lx5 = angle(l_wc, mid);
            double lx6 = angle(sl_sr, l_se);
            double lx8 = angle(l_we, mid);
            double lx7 = angle(l_we, pln);
            double lx9 = angle(l_es, pln);
            double lx10 = angle(l_req, mid);

            //rm_fwd=c1(x1, x2+90,rm_fwd,1,zin1-zin2);
            //rm_bck = c1(x1, x2 + 90, rm_bck, -1,zin1-zin2);
            //rm_start = c1(x1,x2+90,rm_start,-1,Math.Abs(x9-90));
            //rm_fwd = c1(x1-90,x3,rm_fwd,-1);
            //rm_bck = c1(x2 - 90, x3, rm_bck, 1);
            //r_rot = c1(x5 - 90, x4, r_rot, 1,0);
            //r_is = mv(x7 - 90, x8 - 90, r_wrist, r_is, -1);
            //r_rot = mv(x7 - 90, x8 - 90, r_wrist, r_rot, 1);
            //this.semaphore1.Text = "mv : 1st arg = " +  (x7-90)+" 2rd arg = " + (x8-90) ;
            //this.semaphore2.Text = "r_mv = "+ r_is.ToString();
            //this.semaphore3.Text = "zdiff = " + (r_wrist.z - hip_center.z);


            this.semaphore1.Text = "rx 2,9 :" + rx2 + " " + rx9 + " ";
            this.semaphore2.Text = "rx 7,3 : " + rx7 + " " + rx3;
            this.semaphore3.Text = "lx 2,9 :" + lx2 + " " + lx9;
            this.semaphore4.Text = "lx 7,3 : " + lx7 + " " + lx3;
            this.semaphore5.Text = "rx 8,lx 8 : " + rx8 + " " + lx8;
            //DECIDING THE MODE

            if (check(rx2 - 90, rx9, rx7 - 90, rx8) == 1)
            {
                //WHEN BOTH LEFT AND RIGHT HANDS ARE IN POSITION AS REQUIRED (GRIPPER MODE)
                mode = 3;
                lastFrames = 0;
                char x = (char)mode;
                this.serialPort.Write(x.ToString());
            }
            else if (check(rx2 - 90, rx9 - 90, rx7 - 90, rx8) == 1)
            {
                //WHEN RIGHT HAND IS IN POSITION (LONG RANGE MODE)
                mode = 1;
                lastFrames = 0;
                char x = (char)mode;
                this.serialPort.Write(x.ToString());
            }
            else if (check(lx2 - 90, lx9 - 90, lx7 - 90, lx8) == 1)
            {
                //WHEN LEFT HAND IS IN POSTION (FINE CONTROL MODE)
                mode = 2;
                lastFrames = 0;
                char x = (char)mode;
                this.serialPort.Write(x.ToString());
            }
            else
            {
                //lastFrames=0;
            }
            this.semaphore.Text = "MODE : " + mode;
            //WHEN IN MODE 1 i.e. long range motion (* put if condition)

            if (mode == 1)
            {
                //FOR FORWARD MOTION
                if (check(rx2 - 90, rx9 - 90, rx7 - 90, rx8 - 90) == 1 && check(lx2 - 90, lx9 - 90, lx7 - 90, lx8 - 90) == 1)
                {
                    //COMMUNICATE SIGNAL CORRESPONDING TO THIS
                    this.semaphore.Text = "IN MODE 1 FORWARD MOTION DETECTED";
                    char x = (char)8;
                    this.serialPort.Write(x.ToString());
                    lastFrames = 0;
                }

                //FOR STOP
                else if (check(rx2, rx9 - 90, rx7 - 90, rx8) == 1)
                {
                    //STOP
                    this.semaphore.Text = "STOP";
                    char x = (char)5;
                    this.serialPort.Write(x.ToString());
                    lastFrames = 0;
                }

                else if (check(rx2 - 180, rx9 - 90, rx3 - 90, rx7 - 90) == 1 && check(lx2 - 180, lx9 - 90, lx3 - 90, lx7 - 90) == 1)
                {
                    //BACK
                    this.semaphore.Text = "BACK";
                    char x = (char)9;
                    this.serialPort.Write(x.ToString());
                    lastFrames = 0;
                }
                //ANGLE ROTATION
                else if (check(rx2 - 90, 0, rx3 - 180, rx8 - 90) == 1)
                {
                    //SEND ANGLE BETWEEN HAND AND PLN
                    curangle = rx9;
                    if (lastFrames == 0)
                    {
                        initangle = curangle;
                    }
                    lastFrames++;

                    this.semaphore.Text = "IN MODE 1 ANGLE DETECTION : " + send + " s : " + s.ToString();
                    if (lastFrames >= totalFrames)
                    {


                        send = -(Convert.ToInt32(curangle) - Convert.ToInt32(initangle));
                        if (send < 10 && send > -10)
                        {
                            send = 0;
                        }
                        if (send < 0)
                        {
                            send = 45 - send;
                        }
                        send += 10;

                        s = (char)send;

                        this.serialPort.Write(s.ToString());
                        lastFrames = 0;

                    }

                }
                else
                {
                    lastFrames = 0;
                }
            }

            //WHEN IN MODE 2
            else if (mode == 2)
            {
                if (check(rx8 - 90, rx7 - 90, rx9 - 90, 0) == 1 && check(lx8 - 90, lx7 - 90, lx9 - 90, 0) == 1)
                {
                    //RETURN L_WRIST.Z,R_WRIST.Z
                    this.semaphore.Text = "IN MODE 2 : " + r_wrist.z;
                }
            }
            //WHEN IN MODE 3
            else if (mode == 3)
            {
                if (check(rx3 - 180, 0, 0, 0) == 1 && check(lx3 - 180, 0, 0, 0) == 1)
                {
                    lastFrames++;
                    //RETURN rx10,lx10;
                    //RETURN rx9,lx9
                    if (lastFrames >= totalFrames)
                    {
                        lastFrames = 0;
                        if (rx10 > 90) rx10 = 90;
                        int send = Convert.ToInt32(rx10 * (9.0 / 10) + 30);

                        char x = (char)send;
                        this.serialPort.Write(x.ToString());
                        if (rx9 > 90) rx9 = 90;
                        send = Convert.ToInt32(rx9 * (4.0 / 9) + 80);
                        x = (char)send;
                        this.serialPort.Write(x.ToString());
                    }
                    this.semaphore.Text = "IN MODE 3 : rx9,10" + rx9 + " " + rx10;
                }
            }
        }

        //angles constant ew,bodyline 90 ; we,sline 90 /
        //returns +1 if positive motion -1 if -ve motion 0 if no motion 2 if future possibility of motion

        public int checkmotion(double a1, double a2)
        {
            lastFrames++;
            if (a1 < 20 && a2 < 20)
            {
                double temp = Math.Abs(r_wrist.z - rprevz);
                this.semaphore2.Text = "tempval: " + temp + " " + "prevval: " + rprevz;
                if (lastFrames >= totalFrames)
                {
                    lastFrames = 0;
                    fcktemp++;

                    if (temp > thresh)
                    {
                        rprevz = r_wrist.z;
                        return 1;
                    }
                    else if (temp < -thresh)
                    {
                        rprevz = r_wrist.z;
                        return -1;
                    }
                    if (fcktemp < fck)
                    {
                        return 2;
                    }
                    else
                    {
                        fcktemp = 0;
                        return 0;
                    }
                }
                return curx;
            }
            return 0;

        }

        // checks if 3 angles are less around 0 -- returns 1 if true and 0 if false
        public int check(double a1, double a2, double a3, double a4)
        {
            if (Math.Abs(a1) < thresh && Math.Abs(a2) < thresh && Math.Abs(a3) < thresh && Math.Abs(a4) < thresh)
                return 1;
            return 0;
        }

        //passing by reference in c## ?? to set time
        public int c1(double atemp1, double atemp2, int flag, int dir, double z)
        {
            //this.semaphore.Text = "in c1 , flag = " + flag.ToString()+" dir = "+ dir.ToString();

            int inval = flag;
            DateTime curtime = DateTime.Now;
            if (atemp1 < 20 && atemp2 < 130 && atemp2 > 80 && z < 40)
            {
                if (dir == 1)
                {

                    inval = 1;

                }
                else if (flag == 2 && z < 20)
                {

                    inval = 3;
                }
            }
            else if (atemp1 < 20 && atemp2 < 160 && atemp2 > 130 && z < 40)
            {
                if (dir == 1 && flag == 1)
                {
                    inval = 2;

                }
                else if (flag == 1)
                {
                    inval = 2;
                }

            }
            else if (atemp1 < 20 && atemp2 > 160 && z < 40)
            {
                if (dir == 1 && flag == 2)
                {
                    inval = 3;
                }
                else
                {

                    inval = 1;
                }
            }
            return inval;
        }
        public double comp(str s1, str s2)
        {
            double nor = Math.Sqrt(s1.x * s1.x + s1.y * s1.y + s1.z * s1.z);
            return (s1.x * s2.x + s1.y * s2.y + s1.z * s2.z) / nor;
        }
        public int mv(double a1, double a2, str c, int flag, int dir)
        {
            // this.semaphore3.Text = "in mv  :zdiff = " + dir * (c.z-prev.z) ;
            int inval = flag;
            double tempval = Math.Abs(c.z - hip_center.z);
            if (a1 < 40 && a2 < 40 && tempval < 0.14)
            {
                if (dir == -1)
                { inval = 1; }
                else if (flag == 2)
                {
                    inval = 3;
                }
            }
            if (a1 < 40 && a2 < 40 && tempval < 0.23 && tempval > 0.15)
            {
                if (dir == -1 && flag == 1)
                { inval = 2; }
                else if (flag == 1)
                {
                    inval = 2;
                }

            }
            if (a1 < 40 && a2 < 40 && tempval > 0.23)
            {
                if (dir == -1 && flag == 2)
                { inval = 3; }
                else
                {
                    inval = 1;
                }
            }
            return inval;
        }

        public str diff(str x1, str x2)
        {
            str res;
            res.x = x1.x - x2.x;
            res.y = x1.y - x2.y;
            res.z = x1.z - x2.z;
            return res;
        }

        public double angle(str x1, str x2)
        {
            double sum = x1.x * x2.x + x1.y * x2.y + x1.z * x2.z;
            double n1 = Math.Sqrt(x1.x * x1.x + x1.y * x1.y + x1.z * x1.z);
            double n2 = Math.Sqrt(x2.x * x2.x + x2.y * x2.y + x2.z * x2.z);
            sum = Math.Acos(sum / (n1 * n2)) * 180 / Math.PI;
            return sum;

        }

        private double calculateSlope(double x1, double y1, double x2, double y2)
        {
            double slope = 0;

            if (Math.Abs(x2 - x1) != 0)
            {
                slope = Math.Atan2((y2 - y1), (x2 - x1));
            }
            else
            {
                slope = (y1 < y2) ? 0 : Math.PI;
            }

            return Math.Round((slope * 180) / Math.PI, 0);
        }

        private Point getDisplayPosition(Joint joint)
        {
            float depthX, depthY;
            nui.SkeletonEngine.SkeletonToDepthImage(joint.Position, out depthX, out depthY);
            depthX = depthX * 320; //convert to 320, 240 space
            depthY = depthY * 240; //convert to 320, 240 space
            int colorX, colorY;
            ImageViewArea iv = new ImageViewArea();
            // only ImageResolution.Resolution640x480 is supported at this point
            nui.NuiCamera.GetColorPixelCoordinatesFromDepthPixel(ImageResolution.Resolution640x480, iv, (int)depthX, (int)depthY, (short)0, out colorX, out colorY);

            // map back to skeleton.Width & skeleton.Height
            return new Point((int)(skeleton.Width * colorX / 640.0), (int)(skeleton.Height * colorY / 480));
        }

        Polyline getBodySegment(Microsoft.Research.Kinect.Nui.JointsCollection joints, Brush brush, params JointID[] ids)
        {
            PointCollection points = new PointCollection(ids.Length);
            for (int i = 0; i < ids.Length; ++i)
            {
                points.Add(getDisplayPosition(joints[ids[i]]));
            }

            Polyline polyline = new Polyline();
            polyline.Points = points;
            polyline.Stroke = brush;
            polyline.StrokeThickness = 5;
            return polyline;
        }

        void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            SkeletonFrame skeletonFrame = e.SkeletonFrame;
            int iSkeleton = 0;
            Brush[] brushes = new Brush[6];
            brushes[0] = new SolidColorBrush(Color.FromRgb(255, 0, 0));
            brushes[1] = new SolidColorBrush(Color.FromRgb(0, 255, 0));
            brushes[2] = new SolidColorBrush(Color.FromRgb(64, 255, 255));
            brushes[3] = new SolidColorBrush(Color.FromRgb(255, 255, 64));
            brushes[4] = new SolidColorBrush(Color.FromRgb(255, 64, 255));
            brushes[5] = new SolidColorBrush(Color.FromRgb(128, 128, 255));

            skeleton.Children.Clear();
            foreach (SkeletonData data in skeletonFrame.Skeletons)
            {
                if (SkeletonTrackingState.Tracked == data.TrackingState)
                {
                    // Draw bones
                    Brush brush = brushes[iSkeleton % brushes.Length];
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.Spine, JointID.ShoulderCenter, JointID.Head));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.ShoulderCenter, JointID.ShoulderLeft, JointID.ElbowLeft, JointID.WristLeft, JointID.HandLeft));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.ShoulderCenter, JointID.ShoulderRight, JointID.ElbowRight, JointID.WristRight, JointID.HandRight));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.HipLeft, JointID.KneeLeft, JointID.AnkleLeft, JointID.FootLeft));
                    skeleton.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.HipRight, JointID.KneeRight, JointID.AnkleRight, JointID.FootRight));

                    // Draw joints
                    //motion = 0;
                    foreach (Joint joint in data.Joints)
                    {
                        Point jointPos = getDisplayPosition(joint);
                        Line jointLine = new Line();
                        jointLine.X1 = jointPos.X - 3;
                        jointLine.X2 = jointLine.X1 + 6;
                        jointLine.Y1 = jointLine.Y2 = jointPos.Y;
                        jointLine.Stroke = jointColors[joint.ID];
                        jointLine.StrokeThickness = 6;
                        skeleton.Children.Add(jointLine);

                        // TEAM 16_2011
                        // POPULATE DATA
                        // Here populate programming variables with the data processed 
                        if (joint.ID == JointID.WristRight)
                        {
                            r_wrist.x = joint.Position.X;
                            r_wrist.y = joint.Position.Y;
                            r_wrist.z = joint.Position.Z;

                        }
                        if (joint.ID == JointID.ElbowRight)
                        {
                            r_elbow.x = joint.Position.X;
                            r_elbow.y = joint.Position.Y;
                            r_elbow.z = joint.Position.Z;

                        }
                        if (joint.ID == JointID.ShoulderRight)
                        {
                            r_shldr.x = joint.Position.X;
                            r_shldr.y = joint.Position.Y;
                            r_shldr.z = joint.Position.Z;

                        }

                        if (joint.ID == JointID.HandRight)
                        {
                            r_hand.x = joint.Position.X;
                            r_hand.y = joint.Position.Y;
                            r_hand.z = joint.Position.Z;
                        }
                        if (joint.ID == JointID.ShoulderLeft)
                        {
                            l_shldr.x = joint.Position.X;
                            l_shldr.y = joint.Position.Y;
                            l_shldr.z = joint.Position.Z;
                        }
                        if (joint.ID == JointID.WristLeft)
                        {
                            l_wrist.x = joint.Position.X;
                            l_wrist.y = joint.Position.Y;
                            l_wrist.z = joint.Position.Z;

                        }
                        if (joint.ID == JointID.ElbowLeft)
                        {
                            l_elbow.x = joint.Position.X;
                            l_elbow.y = joint.Position.Y;
                            l_elbow.z = joint.Position.Z;

                        }
                        if (joint.ID == JointID.HandLeft)
                        {
                            l_hand.x = joint.Position.X;
                            l_hand.y = joint.Position.Y;
                            l_hand.z = joint.Position.Z;
                        }
                        if (joint.ID == JointID.ShoulderCenter)
                        {
                            shldr_center.x = joint.Position.X;
                            shldr_center.y = joint.Position.Y;
                            shldr_center.z = joint.Position.Z;

                        }

                        if (joint.ID == JointID.HipCenter)
                        {
                            hip_center.x = joint.Position.X;
                            hip_center.y = joint.Position.Y;
                            hip_center.z = joint.Position.Z;
                        }
                    }
                }
                iSkeleton++;
            } // for each skeleton
        }

        void nui_ColorFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            // 32-bit per pixel, RGBA image
            PlanarImage Image = e.ImageFrame.Image;
            video.Source = BitmapSource.Create(
                Image.Width, Image.Height, 96, 96, PixelFormats.Bgr32, null, Image.Bits, Image.Width * Image.BytesPerPixel);
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            nui.Uninitialize();
            // Close the serial port.

            try
            {
                this.serialPort.Close();
            }
            catch (InvalidOperationException)
            {
                MessageBox.Show("An error occurred while closing the port.", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }

            Environment.Exit(0);
        }

        private void semaphore_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void semaphore1_TextChanged(object sender, TextChangedEventArgs e)
        {

        }


    }
}
