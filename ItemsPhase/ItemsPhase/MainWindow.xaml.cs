using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Timers;
using System.Diagnostics;
using System.Threading;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;

using System.Windows.Media.Media3D;
using System.Collections;

namespace ItemsPhase {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window {

        public MainWindow() {
            InitializeComponent();
        }

        //-------------------Global vars and constants--------------------------------

        //GeometryModel3D[] points = new GeometryModel3D[320 * 240];
        int s = 4;
        
        KinectSensor sensor;

        bool b, v = false;
        bool getFrame = false;

        DepthImageFrame depthFrame1;
        short[] depthPixels;

        const int DepWidth = 320;
        const int DepHeight = 240;
        const float MaxDepthDistance = 4095;
        const float MinDepthDistance = 850;
        const float MaxDepthDistanceOffset = MaxDepthDistance - MinDepthDistance;

        const int pointMin = 300; //minimum number of adjacent points to say a set constitutes an item

        //ransac vars
        Random rng = new Random();
        

        const int height = 240, width = 320; //height and width of the depth frames

        SkeletonPoint[,] skPtArray = new SkeletonPoint[width, height];
        string[] lines;
        int linecount = 0;
        float[,] pointarray; //2d array
        string[] temp = new string[3]; //temp after splitting line
        char[] delimiter = { ' ' };
        bool planeFound = false;

        float t = 0.03f; //max error (distance from point to plane, should be metres, but i have no idea, trial and error time
        float g = 0.04f; //+1cm from ransac error, the lowest height an object can be (but wait, I don't really need this since I remove plane points)

        ArrayList finalConsensusSet = new ArrayList(); //the set of points that make up the final set from ransac

        int[,] planeBounds = new int[height, 2]; //store of values to search for items in.
        int[,] planeBoundsBFS = new int[height, 2];
        float[] finalPlane = { 0, 0, 0, 0 }; //var for final plane eqtn

        //item id vars
        Item it = new Item();
        ArrayList itemsList = new ArrayList(); //The global list of items to fill and check
        bool itemsetfound = false;
        //bool

        //----game state variables----
        int[,] hpArray = new int[width, height];
        int[,] dmgArray = new int[width, height]; //each cycle overlay this on top of hpArray to deal damage.
        bool cycleLooping = false; //meaning the game is running
        bool envSet = false;
        bool getGameFrame = false; //use this to grab a frame
        bool breach = false; //game ends if true
        int maxDamage = 16;
        int dmgBonus = 4; //extra thing to add after calculation to ensure damage always gets dealt within radius
        int healRate = 3;
        Stopwatch gameTimer = new Stopwatch(); //every 3 or 5 sec set getGameFrame to true, then do a cycle, set it to false again, or something
        Stopwatch cycleTimer = new Stopwatch(); //for cycles
        Stopwatch tickTimer = new Stopwatch();
        int cycleSec = 1500; //milliseconds per cycle, 1k = 1sec
        int tickSec = 1000;
        int timeMax = 90;
        long timeLeft;
        bool tickflag = false;
        //System.Timers.Timer timer = new System.Timers.Timer(1000);




        //---------------Globals and constants end, methods start---------------------------



        private void Window_Loaded(object sender, RoutedEventArgs e) {
            if (KinectSensor.KinectSensors.Count > 0) {
                /*_sensor = KinectSensor.KinectSensors[0];

                if (_sensor.Status == KinectStatus.Connected) {
                    _sensor.ColorStream.Enable();
                    _sensor.DepthStream.Enable();
                    _sensor.SkeletonStream.Enable();
                    _sensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(_sensor_AllFramesReady);
                    _sensor.Start();

                }*/
            }
            kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser1_KinectSensorChanged);

        } //sets what kinect to use or something

        void kinectSensorChooser1_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e) {
            KinectSensor oldSensor = (KinectSensor)e.OldValue;
            StopKinect(oldSensor);

            KinectSensor newSensor = (KinectSensor)e.NewValue;
            //DepthImageFormat.
            newSensor.ColorStream.Enable();
            newSensor.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30);
            newSensor.DepthStream.Range = DepthRange.Near; //set to near mode
            textBox1.AppendText("Near mode set.\n");
            //newSensor.DepthStream.
            //skeleton stream required for player recognition.
            newSensor.SkeletonStream.Enable();


            newSensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(newSensor_AllFramesReady);
            try {
                newSensor.Start();
            } catch (System.IO.IOException) {

                kinectSensorChooser1.AppConflictOccurred();
            }

            sensor = newSensor;
            //throw new NotImplementedException();
        } //starts the data streams from the kinect

        void newSensor_AllFramesReady(object sender, AllFramesReadyEventArgs e) {
            
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame()) {
                if (depthFrame == null) {
                    return;
                }

                //returns an array of distances and player information
                //byte[] pixels = GenerateColoredBytes(depthFrame);
                //holy processing power batman

                //number of bytes per row withd * 4 (B,G,R,Empty)
                int stride = depthFrame.Width * 4;

                //create image
                //byte[] pixels = GenerateColoredBytes(depthFrame);
                //image1.Source = BitmapSource.Create(depthFrame.Width, depthFrame.Height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);

                if (!v) {
                    textBox1.AppendText("\nDFWidth:Height: " + depthFrame.Width + ":" + depthFrame.Height);
                    depthFrame1 = depthFrame;
                    v = true;
                }
                depthFrame1 = depthFrame;
                //depthFrame1.
                //depthFrame.CopyPixelDataTo(depthPixels);

                //--------------may be very important for weird reason----------
                if (getFrame) {
                    for (int y = 0; y < depthFrame1.Height; y++) {
                        for (int x = 0; x < depthFrame1.Width; x++) {
                            //if (depthFrame1 != null) {
                                skPtArray[(width-1) - x, y] = depthFrame1.MapToSkeletonPoint(x, y); //now have a 2d array of skeleton points, also flips x axis
                            //}
                        }
                    }
                    textBox1.AppendText("\nFrame converted");
                    getFrame = false;
                }



                //-----------------------------------------------------GAME CYCLE LOOP LOGIC THING--------------------------------------------------
                if (cycleLooping) { //this one's for seeing if the thing actually repeats itself
                    if (cycleTimer.ElapsedMilliseconds >= cycleSec) {
                        Console.WriteLine("Cycle completed: " + cycleTimer.ElapsedMilliseconds);
                        //add in a getgameframe = true here somewhere
                        getGameFrame = true;
                        cycleTimer.Restart();
                    }
                    //long x = cycleTimer.ElapsedMilliseconds % tickSec;
                    if (tickTimer.ElapsedMilliseconds >= tickSec) {
                        timeLeft = timeMax - (gameTimer.ElapsedMilliseconds / tickSec);
                        
                        label2.Content = timeLeft;
                        if (timeLeft <= 0) {
                            Console.WriteLine("You win this round!"); //victory
                            textBox1.AppendText("\nYour ability to interface with tangible objects \nhas granted these icy fields victory over the \nmercilessly destructive items!");
                            gameStop();
                        }
                        tickTimer.Restart();
                    }
                }//end if cyclelooping true

                if (getGameFrame && cycleLooping) { //proper-er game loop
                    for (int y = 0; y < depthFrame1.Height; y++) {
                        for (int x = 0; x < depthFrame1.Width; x++) {
                            //if (depthFrame1 != null) {
                            skPtArray[(width - 1) - x, y] = depthFrame1.MapToSkeletonPoint(x, y); //now have a 2d array of skeleton points, also flips x axis
                            //}
                        }
                    }// new useful frame written
                    getGameFrame = false;
                    Console.WriteLine("Game frame converted.");

                    //call game cycle here
                    reIdItems();
                    gameCycle();
                    
                }//end if getgameframe
            }//end using depth frame
        }//end all frames ready

        private byte[] GenerateColoredBytes(DepthImageFrame depthFrame) {

            //get the raw data from kinect with the depth for every pixel
            short[] rawDepthData = new short[depthFrame.PixelDataLength];

            //copy data from depthframe to rawdepthdata
            depthFrame.CopyPixelDataTo(rawDepthData);

            //use depthframe to create the image to display on-screen
            //depthframe contains color information for all pixels in image
            //height x width x 4 (red, green, blue, empty byte)
            Byte[] pixels = new byte[depthFrame.Height * depthFrame.Width * 4];



            //bgr32 - blue green red empty-byte
            //bgra32 - blue green red transparency
            //you must set transparency for bgra as .net defaults a byte to 0 = fully transparent

            //hardcoded locations to blue, green, red (bgr) index positions
            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;
            const int AlphaIndex = 3;

            int depth;
            //loop through all distances
            //pick a rgb color based on distance
            //color index is what we're filling in 


            for (int depthIndex = 0, colorIndex = 0; //init vars
                depthIndex < rawDepthData.Length && colorIndex < pixels.Length;  //conditions
                depthIndex++, colorIndex += 4) {    //incrementals

                //get the player (req's skeleton tracking enabled for values)
                int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;
                //gets depth value
                depth = rawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;
                byte intensity = CalculateIntensityFromDepth(depth);
                //.9M or 2.95' - color everything closer than .9m blue
                if (depth <= 900) {
                    pixels[colorIndex + BlueIndex] = intensity;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 0;
                } else if (depth > 900 && depth < 2000) {
                    //.9m - 2m - color green
                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = intensity;
                    pixels[colorIndex + RedIndex] = 0;
                } else if (depth > 2000) { //further than 2m
                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = intensity;

                }
                //equal coloring for monochromatic histogram

                /*pixels[colorIndex + BlueIndex] = intensity;
                pixels[colorIndex + GreenIndex] = intensity;
                pixels[colorIndex + RedIndex] = intensity;*/

                /*if (player > 0) {
                    pixels[colorIndex + BlueIndex] = Colors.Gold.B;
                    pixels[colorIndex + GreenIndex] = Colors.Gold.G;
                    pixels[colorIndex + RedIndex] = Colors.Gold.R;

                }*/

            }

            if (!b) {
                textBox1.AppendText("rawDepthData: " + rawDepthData.Length + "\npixels: " + pixels.Length);
                textBox1.AppendText("\n1110PixelDepth: " + rawDepthData[1110] + "\n1110PixelColor: " + pixels[1110 * 4] + "," + pixels[1110 * 4 + 1] + "," + pixels[1110 * 4 + 2] + "," + pixels[1110 * 4 + 3]);
                b = true;
            }
            /*pixels[4440] = 0;
            pixels[4441] = 0;
            pixels[4442] = 0;*/

            return pixels;
        }

        public static byte CalculateIntensityFromDepth(int distance) {
            //formula for calculating monochrome intensity for histogram
            return (byte)(255 - (255 * Math.Max(distance - MinDepthDistance, 0) / (MaxDepthDistanceOffset)));
        }


        /**
         * Colour a 320 x 240 bitmap by what state each data point is in {-1 ... 4}
         */
        byte[] drawFieldInit(int[,] statearray) {
            Byte[] points = new byte[width * height*4]; //1 byte per colour per point
            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;
            const int AlphaIndex = 3;

            int colourIndex = 0;//gotta put this on the outside otherwise it colours the same line over and over >.>
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width && colourIndex < points.Length; x++, colourIndex += 4) { //writing into byte array too, so colour index
                    if (statearray[x, y] == -1) { //out of bounds pt - black
                        points[colourIndex + BlueIndex] = 0;
                        points[colourIndex + GreenIndex] = 0;
                        points[colourIndex + RedIndex] = 0;
                        //points[colourIndex + AlphaIndex] = 255;
                    } else if (statearray[x, y] == 0) { //desk pt - blue
                        points[colourIndex + BlueIndex] = 255;
                        points[colourIndex + GreenIndex] = 0;
                        points[colourIndex + RedIndex] = 0;
                        //points[colourIndex + AlphaIndex] = 255;
                    } else if (statearray[x, y] == 1) { //unvisited -purple
                        points[colourIndex + BlueIndex] = 120;
                        points[colourIndex + GreenIndex] = 0;
                        points[colourIndex + RedIndex] = 120;
                        //points[colourIndex + AlphaIndex] = 255;
                    } else if (statearray[x, y] == 2) { //visited - orange
                        points[colourIndex + BlueIndex] = 0;
                        points[colourIndex + GreenIndex] = 120;
                        points[colourIndex + RedIndex] = 120;
                        //points[colourIndex + AlphaIndex] = 255;
                    } else if (statearray[x, y] == 3) { //item point - green
                        points[colourIndex + BlueIndex] = 0;
                        points[colourIndex + GreenIndex] = 255;
                        points[colourIndex + RedIndex] = 0;
                        //points[colourIndex + AlphaIndex] = 255;
                    } else if (statearray[x, y] == 4) { //outlyer point - red
                        points[colourIndex + BlueIndex] = 0;
                        points[colourIndex + GreenIndex] = 0;
                        points[colourIndex + RedIndex] = 255;
                        //points[colourIndex + AlphaIndex] = 255;
                    } //end colouring

                }//end x
            }//end y

            return points;
        }



        

        //---------------------------------------------Button onclicks here---------------------------------------------------

        /**
         * "Set Tabletop"
         * RANSAC and Edge finding
         */
        private void button1_Click(object sender, RoutedEventArgs e) { //do some ransac and edges
            int frameno;
            int x =160, y = 120;
            
            textBox1.AppendText("\nFrame gathered" + skPtArray[x, y].Z + ", x:"+x+" y:"+y);
            
            ransac();

            //edgeFinderLinearSP();
            edgeFinderBFSfast();
            planeFound = true;
        } //end button 1

        /**
         * "Detect Items"
         * Determine the item locations
         */
        private void button2_Click(object sender, RoutedEventArgs e) { //detect items button
            //identifyItems();
            if (planeFound) {
                idItemFast();
                itemsetfound = true;
            } else {
                Console.WriteLine("Find the game field first.");
            }
        } //end button 2

        private void button3_Click(object sender, RoutedEventArgs e) { //initialize game environment button
            if (itemsetfound) {
                gameEnvInit();
            } else {
                Console.WriteLine("Identify items first.");
            }
        } //end button 3 - game env setup

        private void button4_Click(object sender, RoutedEventArgs e) { //start game cycle button
            if (envSet) {
                gameStart();
            } else {
                Console.WriteLine("Initialise game environment first.");
            }
        } //end button 4 - game start


        private void button5_Click(object sender, RoutedEventArgs e) { //get a frame
            getFrame = true;
        } //end button 5 - get a frame

        private void button6_Click(object sender, RoutedEventArgs e) { //reassess items
            //reassessItems();           
            if (itemsetfound) {
                reIdItems();
            } else {
                Console.WriteLine("Identify items first.");
            }
        } //end button 6 - reassess items

        private void button7_Click(object sender, RoutedEventArgs e) { //step through a cycle
            if (envSet) {
                Console.WriteLine("Game cycle stepping...");
                Stopwatch stopwatch = new Stopwatch();
                stopwatch.Start();

                reIdItems();
                gameCycle();

                stopwatch.Stop();
                Console.WriteLine("Game cycle took: " + stopwatch.Elapsed);
            } else {
                Console.WriteLine("Initialise game environment first.");
            }
        } //end button 7 - step cycle

        //stop everything
        private void button8_Click(object sender, RoutedEventArgs e) {
            gameStop();
        } //end button 8 - stop


        /// <summary>
        /// ///////////////////////Stack of other random functions
        /// </summary>
        /// <param name="sensor"></param>


        void StopKinect(KinectSensor sensor) {
            if (sensor != null) {
                if (sensor.IsRunning) {

                    sensor.Stop();

                    if (sensor.AudioSource != null) {
                        sensor.AudioSource.Stop();
                    }
                }
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e) {
            StopKinect(kinectSensorChooser1.Kinect);

        }



        //------------------------------RANSAC + Edge finding here---------------------------------------------

        /**
         * RANSAC Skeleton Points
         */
        void ransac() {
            textBox1.AppendText("\n-----Ransacing...-----\n");
            Console.WriteLine("\n-----Ransacing...-----\n");

            //restrict the bounds of which points can be used as starting locations for ransac to limit floor being determined as desk when there's actually a desk there already
            int xrestrict = 70;
            int yrestrict = 60;

            int n = 3;
            //int k = 1000;
            int k = 150; //testing purposes
            int d = 15000;

            ArrayList bestConsensusSet = new ArrayList(); //they dont have types!?
            ArrayList consensusSet = new ArrayList();
            float bestError = float.PositiveInfinity; //start error at the max value possible
            float sumError = 0;
            float pointError = 0;
            float thisError = float.PositiveInfinity; //the error for the current iteration
            //float[] bestPlane = new float[4]; //bestmodel
            float[] bestPlane = { 0, 0, 0, 0 }; //bestmodel
            float[] maybePlane = { 0, 0, 0, 0 }; //maybemodel
            float[] thisPlane = { 0, 0, 0, 0 }; //thismodel, also the model fitted to all points in consensus set (whatever that means)

            //data = skCoordsArray = a 2d array widthxheight 320x240 of skeleton points
            SkeletonPoint spointinQ = new SkeletonPoint();
            SkeletonPoint spoint1 = new SkeletonPoint();
            SkeletonPoint spoint2 = new SkeletonPoint();
            SkeletonPoint spoint3 = new SkeletonPoint();
            SkeletonPoint spointx = new SkeletonPoint();
            int[,] maybeInlierSP = new int[n, 2]; //3x2, 2 = x,y coords

            //textBox1.AppendText("\n-----Ransacing...-----\n");

            for (int i = 0; i < k; i++) { //for each iteration
                consensusSet.Clear();
                sumError = 0;
                Console.Write("" + i + ",");

                //randomly sample 3 points
                maybeInlierSP[0, 0] = rng.Next(xrestrict, width - xrestrict); //x
                maybeInlierSP[0, 1] = rng.Next(yrestrict, height - yrestrict); //y
                maybeInlierSP[1, 0] = rng.Next(xrestrict, width - xrestrict);
                maybeInlierSP[1, 1] = rng.Next(yrestrict, height - yrestrict);
                maybeInlierSP[2, 0] = rng.Next(xrestrict, width - xrestrict);
                maybeInlierSP[2, 1] = rng.Next(yrestrict, height - yrestrict);

                //ensure no duplicate points
                while (maybeInlierSP[0, 0] == maybeInlierSP[1, 0] && maybeInlierSP[0, 1] == maybeInlierSP[1, 1]) {
                    maybeInlierSP[1, 0] = rng.Next(40, width - 40);
                    maybeInlierSP[1, 1] = rng.Next(40, height - 40);
                }
                while (maybeInlierSP[0, 0] == maybeInlierSP[2, 0] && maybeInlierSP[0, 1] == maybeInlierSP[2, 1]
                    || maybeInlierSP[1, 0] == maybeInlierSP[2, 0] && maybeInlierSP[1, 1] == maybeInlierSP[2, 1]) {
                    maybeInlierSP[2, 0] = rng.Next(40, width - 40);
                    maybeInlierSP[2, 1] = rng.Next(40, height - 40);
                }

                spoint1 = skPtArray[maybeInlierSP[0,0], maybeInlierSP[0,1]]; //maybe pt 1
                spoint2 = skPtArray[maybeInlierSP[1,0], maybeInlierSP[1,1]]; //2
                spoint3 = skPtArray[maybeInlierSP[2,0], maybeInlierSP[2,1]]; //3

                maybePlane = fitPlane(spoint1, spoint2, spoint3); //get plane equation

                consensusSet.Add(new Coord(maybeInlierSP[0, 0], maybeInlierSP[0, 1])); //add first 3 indices to consensus set
                consensusSet.Add(new Coord(maybeInlierSP[1, 0], maybeInlierSP[1, 1]));
                consensusSet.Add(new Coord(maybeInlierSP[2, 0], maybeInlierSP[2, 1]));

                for (int x = 0; x < width; x++) { //for each row
                    for (int y = 0; y < height /*&& maybe something so we don't have replica points*/; y++) { //and each column
                        if (skPtArray[x, y].Z > 0) { //if the point is in front of the camera
                            spointinQ = skClone(skPtArray[x, y]); //make copy of this point

                            pointError = pointPlaneDistSP(maybePlane, spointinQ); //see how far it is from the kinect

                            if (pointError < t) { //if point to plane is within threshold
                                consensusSet.Add(new Coord(x, y));
                                sumError += pointError;
                            }//end if point close enough
                        } //end if in front of camera
                    } //end y
                }//end x

                if (consensusSet.Count > d) { //we have enough points
                    thisError = sumError / consensusSet.Count; //now unneeded I think

                    if (consensusSet.Count > bestConsensusSet.Count) { //if we have a new best sized set
                        bestPlane[0] = maybePlane[0];
                        bestPlane[1] = maybePlane[1];
                        bestPlane[2] = maybePlane[2];
                        bestPlane[3] = maybePlane[3];

                        //printout a bunch of stuff
                        Console.WriteLine("\nBest current plane (" + i + "): " + bestPlane[0] + "x + " + bestPlane[1] + "y + " + bestPlane[2] + "z + " + bestPlane[3] + " = 0");
                        Console.WriteLine("Random point indices: " + maybeInlierSP[0, 0] + ":" + maybeInlierSP[0, 1] +
                            ", " + maybeInlierSP[1, 0] + ":" + maybeInlierSP[1, 1] + ", "
                            + maybeInlierSP[2, 0] + ":" + maybeInlierSP[2, 1] + ";" + "\nThe three coordinates");

                        Console.WriteLine("Aligning points: " + consensusSet.Count);
                        Console.WriteLine("Error: " + thisError);

                        bestConsensusSet.Clear();
                        foreach (Coord s in consensusSet) {
                            bestConsensusSet.Add(s.clone());
                        }
                        planeFound = true;
                        bestError = thisError;

                    } //end if this set is bigger than previous best set
                } //end of if we have consensus size # of points
            }//end iterations

            finalPlane[0] = bestPlane[0];
            finalPlane[1] = bestPlane[1];
            finalPlane[2] = bestPlane[2];
            finalPlane[3] = bestPlane[3];

            foreach (Coord c in bestConsensusSet) {
                finalConsensusSet.Add(c.clone()); //fill in final consensus set
            }
            bestConsensusSet.Clear();
            consensusSet.Clear();
            Console.WriteLine("\nFinal Plane Equation: " + finalPlane[0] + "x + " + finalPlane[1] + "y + " + finalPlane[2] + "z + " + finalPlane[3] + " = 0");

        }//end ransacSP


        /**
         * Fit a plane of form ax + by + cz +d = 0 given 3 xyz coords
         */
        float[] fitPlane(SkeletonPoint pt1, SkeletonPoint pt2, SkeletonPoint pt3) {
            float[] plane = new float[4];
            int a = 0, b = 1, c = 2, d = 3;
            //pt<123>[x,y,z:0,1,2]

            //calculate plane given points
            plane[a] = pt1.Y * (pt2.Z - pt3.Z) + pt2.Y * (pt3.Z - pt1.Z) + pt3.Y * (pt1.Z - pt2.Z);
            plane[b] = pt1.Z * (pt2.X - pt3.X) + pt2.Z * (pt3.X - pt1.X) + pt3.Z * (pt1.X - pt2.X);
            plane[c] = pt1.X * (pt2.Y - pt3.Y) + pt2.X * (pt3.Y - pt1.Y) + pt3.X * (pt1.Y - pt2.Y);
            // invert for ... = d (ie comment out the -1)
            plane[d] = (-1) * (pt1.X * (pt2.Y * pt3.Z - pt3.Y * pt2.Z) + pt2.X * (pt3.Y * pt1.Z - pt1.Y * pt3.Z) + pt3.X * (pt1.Y * pt2.Z - pt2.Y * pt1.Z));


            return plane;
        }//end fitPlaneSP method

        /**
         * Find the smallest (ie perpendicular) distance from a given point to a given plane.
         */
        float pointPlaneDistSP(float[] plane, SkeletonPoint point) {
            float result = 0;
            float topline = 0;
            float botline = 0;
            //D = |ax + by + cz + d| / sqrt(a^2 + b^2 + c^2)
            //magic
            //better ensure this value comes out as absolute
            topline = plane[0] * point.X + plane[1] * point.Y + plane[2] * point.Z + plane[3];
            topline = Math.Abs(topline);
            botline = (float)(Math.Pow(plane[0], 2) + Math.Pow(plane[1], 2) + Math.Pow(plane[2], 2));
            botline = (float)Math.Sqrt(botline);

            if (botline != 0) {
                result = topline / botline;
            }
            return result;
        } //end pointPlaneDistSP method


        /**
         * Figure out which points are the edges of the desk via BFS using the finalConsensusSet provided by Ransac
         */
        void edgeFinderBFSfast() {
            //know finalconsensusset<coord> and also have planeboundsbfs[height,2]
            //fill in planeboundsbfs
            int[,] stateArray = new int[width, height]; //map fcs coords to this
            const int np = -1; //not point
            const int dp = 0; //desk point <-- the important one, want all to be this.
            const int uvp = 1; //unvisited point
            const int vp = 2; //visited point
            const int op = 4; //outlier point

            bool leftbound = false;
            bool rightbound = false;

            ArrayList currSet = new ArrayList();
            ArrayList bestSet = new ArrayList();
            ArrayList ptQ = new ArrayList();
            Coord cord = new Coord();

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    stateArray[x, y] = np; //establish all points inside state array as stuff I don't care about initially
            }  }
            
            foreach (Coord c in finalConsensusSet) {
                stateArray[c.x, c.y] = uvp; //say there's an unvisited point there
            }

            //for every point, if 1 then bfs, if anything else, ignore
            bestSet.Clear(); //just in case
            Console.WriteLine("Starting BFS for edge detection:");

            for (int y = 0; y < height; y++) { //for every row
                for (int x = 0; x < width; x++) {
                    //if (stateArray[x, y] == vp) { //check if it's a point that's been visited but not assigned, which shouldn't be the case
                    //    Console.WriteLine(x + "," + y + " is Visited for some reason in edge finder");
                    //}

                    if (stateArray[x, y] == uvp) { //something to investigate!
                        Console.WriteLine("Found a point to BFS from.");
                        //ensure things are clear
                        currSet.Clear();
                        ptQ.Clear();

                        ptQ.Add(new Coord(x, y)); //add that position to the queue
                        int debugint = 0;

                        while (ptQ.Count > 0 ) { //while there's something in ptQ
                            //debugint++;
                            //if (debugint % 1000 == 0) {
                            //    Console.WriteLine(debugint + ", " +ptQ.Count);
                            //}
                            cord = ((Coord)ptQ[0]).clone(); //get the first thing in the queue out of it
                            stateArray[cord.x, cord.y] = vp; //label that point as visited
                            currSet.Add(new Coord(cord.x, cord.y)); //add the coord to the current set of things <--this right here was using wrong x/y values for ages
                            ptQ.RemoveAt(0); //get it the hell out of that queue!

                            //now for adjacent points!
                            //top
                            if (cord.y - 1 >= 0) { //if point is inside array bounds
                                if (stateArray[cord.x, cord.y - 1] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x, cord.y - 1)); //then add to the ptQ
                                    stateArray[cord.x, cord.y -1] = vp; //label that point as visited
                                }
                            }
                            //right
                            if (cord.x + 1 < width) { //if point is inside array bounds
                                if (stateArray[cord.x + 1, cord.y] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x + 1, cord.y)); //then add to the ptQ
                                    stateArray[cord.x +1, cord.y] = vp; //label that point as visited
                                }
                            }
                            //down
                            if (cord.y + 1 < height) { //if point is inside array bounds
                                if (stateArray[cord.x, cord.y + 1] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x, cord.y + 1)); //then add to the ptQ
                                    stateArray[cord.x, cord.y +1] = vp; //label that point as visited
                                }
                            }
                            //left
                            if (cord.x - 1 >= 0) { //if point is inside array bounds
                                if (stateArray[cord.x - 1, cord.y] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x - 1, cord.y)); //then add to the ptQ
                                    stateArray[cord.x -1, cord.y] = vp; //label that point as visited
                                }
                            }//end adj pt determination
                            //Console.WriteLine();
                            //foreach (Coord c in ptQ) {
                            //    Console.Write(c.getStr() + " | ");
                            //}
                        }//end BFS while loop, currSet contains entire set of points

                        
                        

                        Console.WriteLine("BFS while loop completed.");
                        //if currset.count > bestset.count then for every point in bestset, set to outlier point
                        if (currSet.Count > bestSet.Count) {
                            debugint = 0;
                            foreach (Coord c in bestSet) {
                                stateArray[c.x, c.y] = op;
                            }
                            bestSet.Clear();
                            foreach (Coord c in currSet) {
                                bestSet.Add(c.clone()); //copy new best set over
                                stateArray[c.x, c.y] = dp; //set it as a desk point (what we want)
                                debugint++;
                            }
                            Console.WriteLine(debugint + " Points set to desk points.");

                        } else { //then the current set is weak and can't compare to the true potential of the desk
                            debugint = 0;
                            foreach (Coord c in currSet) {
                                stateArray[c.x, c.y] = op; //then this set is outliers too
                                debugint++;
                            }
                            Console.WriteLine(debugint + " Points set to outliers.");
                        }
                        //for (int yy = 0; yy < height; yy++) { //from top of frame to bottom
                        //    Console.WriteLine();
                        //    for (int xx = 0; xx < width; xx++) { //from left to right
                        //        if (stateArray[xx,yy] == -1) {
                        //            Console.Write("-");
                        //        } else {
                        //            Console.Write(stateArray[xx,yy]);
                        //        }
                        //    }
                        //}
                    }//end if we've found a point to BFS

                } //end x loop
            } //end y loop

            Console.WriteLine("Edge detection BFS finished.");

            //forward search
            for (int y = 0; y < height; y++) { //from top of frame to bottom
                leftbound = false; //for the start of each row
                for (int x = 0; x < width && leftbound == false; x++) { //from left to right, while the left bound hasnt been found
                    if (stateArray[x, y] == dp) {
                        planeBoundsBFS[y, 0] = x;
                        leftbound = true;
                    }
                }
            }//end forward search

            //backward search
            for (int y = height - 1; y >= 0; y--) { //from bottom to top
                rightbound = false;
                for (int x = width - 1; x >= 0 && rightbound == false; x--) { //right to left
                    if (stateArray[x, y] == dp) {
                        planeBoundsBFS[y, 1] = x;
                        rightbound = true;
                    }
                }
            }//end backward search

            //for (int i = 0; i < height; i++) {
            //    Console.WriteLine(i + ": " + planeBoundsBFS[i, 0] + "," + planeBoundsBFS[i, 1]);
            //}

            byte[] points = drawFieldInit(stateArray);
            int stride = width * 4;
            //image1.
            image1.Source = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, points, stride);
            //image2.Source = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, points, stride);

        }//end edgefinderbfsfast


        /**
         * Attempt to find the edges of the plane via linear scan of the point set.
         * May require realignment of points into 2d array.
         */
        void edgeFinderLinearSP() {
            int i = 0, j = 0;
            int offset;
            int coordX = 0, coordY = 0;
            float[] pointinQ = new float[3];
            SkeletonPoint spointinQ = new SkeletonPoint();
            bool leftbound = false, rightbound = false;
            float pointError = float.PositiveInfinity;
            //float t = 0.04f; //nearness a point has to be, redefined as global


            for (i = 0; i < height; i++) {//set all to initial values
                planeBounds[i, 0] = -1;
                planeBounds[i, 1] = -1;
            }

            j = width;
            offset = 0;

            //forward search
            for (int y = 0; y < height; y++) { //from top of frame to bottom
                leftbound = false; //for the start of each row
                for (int x = 0; x < width && leftbound == false; x++) { //from left to right, while the left bound hasnt been found
                    spointinQ = skClone(skPtArray[x, y]); //clone the point

                    pointError = pointPlaneDistSP(finalPlane, spointinQ);

                    if (pointError < t) {
                        planeBounds[y, 0] = x;
                        leftbound = true;
                    }
                }
            }//end forward search

            //backward search
            for (int y = height - 1; y >= 0; y--) { //from bottom to top
                rightbound = false;
                for (int x = width - 1; x >= 0 && rightbound == false; x--) { //right to left
                    spointinQ = skClone(skPtArray[x, y]); //clone the point

                    pointError = pointPlaneDistSP(finalPlane, spointinQ);

                    if (pointError < t) {
                        planeBounds[y, 1] = x;
                        rightbound = true;
                    }

                }
            }//end backward search


            //colour the image1 box...
            const int uvp = 1;
            const int dp = 0;
            const int np = -1;
            int[,] stateArray = new int[width, height];
            float ptError = float.PositiveInfinity;
            //for every point, determine if inside bounds, if so, determine if desk or potential item point
            for (int y = 0; y < height; y++) { //for every row aka down
                for (int x = 0; x < width; x++) { //for every column aka right (can't tell if memory optimal way)
                    if (x > planeBounds[y, 0] && x < planeBounds[y, 1]) { //if within the bounds of the desk
                        //spointinQ = skClone(skPtArray[x, y]);
                        //now to get rid of all the points, that are within the same error as ransac, of the plane/desk, leaving with only item points or outliers
                        if (skPtArray[x, y].Z > 0) { //in front of camera
                            ptError = pointPlaneDistSP(finalPlane, skPtArray[x, y]);
                            if (ptError > g) { //4cm from plane (likely to be an item)
                                //refinedPoints.Add(new Coord(x, y));
                                stateArray[x, y] = uvp;
                                //Console.Write("1");
                            } else { //end likely to be item, else probably desk
                                stateArray[x, y] = dp;
                                //Console.Write("0");
                            } //end not an item (desk)
                        }
                    } else {//end if in bounds
                        stateArray[x, y] = np;
                        //Console.Write("-");
                    } //end out of bounds
                }//end x
                Console.WriteLine();
            }//end y
            byte[] points = drawFieldInit(stateArray);
            int stride = width * 4;
            image1.Source = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, points, stride);



        } //end linear search


       


        //------------------------------------old stuff---------------------------------------------------
        
        

        /**
         * Find the adjacent point indexes to a given point.
         * Returns size 4 array of adjacent point indices.
         * Point will be -1 if the adjacent point isn't actually a point in the 2d grid.
         */
        int[] adjPointFinder(int pt) {
            int[] result = { -1, -1, -1, -1 }; //top, right, bot, left (clockwise)

            //get top/bot points
            if (pt >= width && pt < (linecount - width)) { //if not on first line or last line, therefore top and bot are +/- 320
                result[0] = pt - width; //top
                result[2] = pt + width; //bot
            } else if (pt < width) { //if on first line, so only bot has value
                result[2] = pt + width; //bot
            } else { //if on last line, only top has value
                result[0] = pt - width; //top
            }

            //get left/right side points
            if ((pt % width) > 0 && (pt % width) < (width - 1)) { //if the point isnt on the left or right edge of the line, then left/right = pt +/-1
                result[1] = width + 1; //right
                result[3] = width - 1; //left
            } else if ((pt % width) == 0) { //left side
                result[1] = width + 1; //right
            } else { //right side
                result[3] = width - 1; //left
            }

            return result;
        }
       



        //--------------------------Item detection phase---------------------------------

        /**
         * Faster item id
         */
        void idItemFast() {
            //planeBounds[height,2] = edges of desk
            //skPtArray[width/x, height/y] = set of skeleton points
            //pointMin = number of points required to constitute an item

            //so we have the set of points for some frame, not the frame that was used for ransac either
            //sidenote: figure out a way to get 2d array for this thing
            const int np = -1; //non valid point
            const int dp = 0; //desk point
            const int uvp = 1; //unvisited, potential item point
            const int vp = 2; //visited, potential item point
            const int ip = 3; //item point confirmed
            const int op = 4; //outlyer point

            ArrayList ptQ = new ArrayList(); //point queue
            ArrayList itemSet = new ArrayList();
            int[,] stateArray = new int[width, height];

            Coord cord = new Coord();
            
            float ptError = float.PositiveInfinity;

            //item init values
            int[] cent = new int[2];
            float vol = 0.0f;

            Item it;
            Stopwatch stopwatch = new Stopwatch();

            itemsList.Clear(); //ensure we make a new set of items

            Console.WriteLine("-----Detecting items!-----");
            textBox1.AppendText("\n-----Detecting items!-----");
            stopwatch.Start();

            //for every point, determine if inside bounds, if so, determine if desk or potential item point
            for (int y = 0; y < height; y++) { //for every row aka down
                for (int x = 0; x < width; x++) { //for every column aka right (can't tell if memory optimal way)
                    if (x > planeBoundsBFS[y, 0] && x < planeBoundsBFS[y, 1]) { //if within the bounds of the desk
                        //spointinQ = skClone(skPtArray[x, y]);
                        //now to get rid of all the points, that are within the same error as ransac, of the plane/desk, leaving with only item points or outliers
                        if (skPtArray[x,y].Z > 0) { //in front of camera
                            ptError = pointPlaneDistSP(finalPlane, skPtArray[x, y]);
                            if (ptError > g) { //4cm from plane (likely to be an item)
                                //refinedPoints.Add(new Coord(x, y));
                                stateArray[x, y] = uvp;
                            } else { //end likely to be item, else probably desk
                                stateArray[x, y] = dp;
                            } //end not an item (desk)
                        }
                    } else {//end if in bounds
                        stateArray[x, y] = np;
                    } //end out of bounds
                }//end x
            }//end y
            //now have a set of points classified by item potential

            //for each point, ignore if -1, 3 or 4, post message if 2 because that's odd, if 1 then add it to ptQ and also itemSet and flag as 2, then while ptQ != empty, find
            //adjacent points and add those to ptQ and also itemSet + flag as 2.
            //when ptQ == empty then take itemSet, see if it's big enough to qualify as an item, if so, form new item and add to itemarray

            for (int y = 0; y < height; y++) { //for every row
                for (int x = 0; x < width; x++) {
                    if (stateArray[x, y] == vp) { //check if it's a point that's been visited but not assigned, which shouldn't be the case
                        Console.WriteLine(x + "," + y + " is Visited for some reason");
                    }

                    if (stateArray[x, y] == uvp) { //something to investigate!
                        itemSet.Clear(); //ensure things are clear
                        ptQ.Clear();

                        ptQ.Add(new Coord(x, y)); //add that position to the queue
                        
                        while (ptQ.Count > 0) { //while there's something in ptQ
                            cord = ((Coord)ptQ[0]).clone(); //get the first thing in the queue out of it
                            stateArray[cord.x, cord.y] = vp; //label that point as visited
                            itemSet.Add(new Coord(cord.x, cord.y)); //add the coord to the itemSet
                            ptQ.RemoveAt(0); //get it the hell out of that queue!

                            //now for adjacent points!
                            //top
                            if (cord.y-1 >= 0 ) { //if point is inside array bounds
                                if (stateArray[cord.x, cord.y-1] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x, cord.y-1)); //then add to the ptQ
                                    stateArray[cord.x, cord.y-1] = vp; //label that point as visited
                                }
                            }
                            //right
                            if (cord.x + 1 < width) { //if point is inside array bounds
                                if (stateArray[cord.x + 1, cord.y] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x + 1, cord.y)); //then add to the ptQ
                                    stateArray[cord.x+1, cord.y] = vp; //label that point as visited
                                }
                            }
                            //down
                            if (cord.y + 1 < height) { //if point is inside array bounds
                                if (stateArray[cord.x, cord.y+1] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x, cord.y+1)); //then add to the ptQ
                                    stateArray[cord.x, cord.y+1] = vp; //label that point as visited
                                }
                            }
                            //left
                            if (cord.x - 1 >= 0) { //if point is inside array bounds
                                if (stateArray[cord.x - 1, cord.y] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x - 1, cord.y)); //then add to the ptQ
                                    stateArray[cord.x-1, cord.y] = vp; //label that point as visited
                                }
                            }//end adj pt determination
                        }//end BFS while loop, itemSet contains entire set of points

                        if (itemSet.Count >= pointMin) { //we've got enough points to make an item
                            vol = calcVolumeFast((Coord[])itemSet.ToArray(typeof(Coord))); //calculate volume of object
                            cent = calcCentroid((Coord[])itemSet.ToArray(typeof(Coord))); //calculate centroid of object
                            itemsList.Add(new Item(itemSet.Count, vol, cent[0], cent[1]));

                            for (int i = 0; i < itemSet.Count; i++) { //label all the points as item points (3)
                                stateArray[((Coord)itemSet[i]).x, ((Coord)itemSet[i]).y] = ip; //set points to 3s
                            }

                            Console.WriteLine("New Item found! " + itemSet.Count + ", " + vol + ", " + cent[0] + ", " + cent[1]);
                            textBox1.AppendText("\nNew Item found: " + itemSet.Count + ", " + vol + ", " + cent[0] + ", " + cent[1]);
                            itemSet.Clear(); //clear out the itemset
                        } else { //otherwise there arent enough points, so flag those found as outlyers (4)
                            for (int i = 0; i < itemSet.Count; i++) { //label all the points as outlyers
                                stateArray[((Coord)itemSet[i]).x, ((Coord)itemSet[i]).y] = op; //set points to 4s
                            }
                        } //end classifying not points

                    }//end if we've found a point to BFS


                } //end x loop
            } //end y loop

            //how long did it take?
            stopwatch.Stop();
            Console.WriteLine("Time elapsed: " + stopwatch.Elapsed);

            
            float maxVol = 0.0f;
            foreach (Item i in itemsList) { //figure out which item is the largest
                if (i.volume > maxVol) {
                    maxVol = i.volume;
                }
            }
            foreach (Item i in itemsList) { //give % weighting to each item in set
                i.relWeight = i.volume / maxVol;
            }


            //what state are the points in now?
            byte[] points = drawFieldInit(stateArray);
            int stride = width * 4;
            image1.Source = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, points, stride);

            //all adjacent points, if sufficient #, become new item by performing summations of points
            foreach (Item i in itemsList) {
                Console.WriteLine(i.getString());
            }
        }//end idItemFast


        /**
         * Faster item reassessment
         */
        void reIdItems() {
            //planeBounds[height,2] = edges of desk
            //skPtArray[width/x, height/y] = set of skeleton points
            //pointMin = number of points required to constitute an item

            //so we have the set of points for some frame, not the frame that was used for ransac either
            //sidenote: figure out a way to get 2d array for this thing
            const int np = -1; //non valid point
            const int dp = 0; //desk point
            const int uvp = 1; //unvisited, potential item point
            const int vp = 2; //visited, potential item point
            const int ip = 3; //item point confirmed
            const int op = 4; //outlyer point

            const float comparisonError = 0.15f; // % error that an item can match to <----- all the way important

            ArrayList ptQ = new ArrayList(); //point queue
            ArrayList itemSet = new ArrayList();
            int[,] stateArray = new int[width, height];

            Coord cord = new Coord();

            float ptError = float.PositiveInfinity;
            

            //item init values
            int[] cent = new int[2];
            float vol = 0.0f;

            Item it;
            Stopwatch stopwatch = new Stopwatch();

            //itemsList.Clear(); //ensure we make a new set of items <--not for reassessment! breaks everything!

            Console.WriteLine("-----Reassessing items!-----");
            //textBox1.AppendText("\n-----Reassessing items!-----");
            stopwatch.Start();

            //for every point, determine if inside bounds, if so, determine if desk or potential item point
            for (int y = 0; y < height; y++) { //for every row aka down
                for (int x = 0; x < width; x++) { //for every column aka right (can't tell if memory optimal way)
                    if (x > planeBoundsBFS[y, 0] && x < planeBoundsBFS[y, 1]) { //if within the bounds of the desk
                        //spointinQ = skClone(skPtArray[x, y]);
                        //now to get rid of all the points, that are within the same error as ransac, of the plane/desk, leaving with only item points or outliers
                        if (skPtArray[x, y].Z > 0) { //in front of camera
                            ptError = pointPlaneDistSP(finalPlane, skPtArray[x, y]);
                            if (ptError > g) { //4cm from plane (likely to be an item)
                                //refinedPoints.Add(new Coord(x, y));
                                stateArray[x, y] = uvp;
                            } else { //end likely to be item, else probably desk
                                stateArray[x, y] = dp;
                            } //end not an item (desk)
                        }
                    } else {//end if in bounds
                        stateArray[x, y] = np;
                    } //end out of bounds
                }//end x
            }//end y
            //now have a set of points classified by item potential

            //for each point, ignore if -1, 3 or 4, post message if 2 because that's odd, if 1 then add it to ptQ and also itemSet and flag as 2, then while ptQ != empty, find
            //adjacent points and add those to ptQ and also itemSet + flag as 2.
            //when ptQ == empty then take itemSet, see if it's big enough to qualify as an item, if so, form new item and add to itemarray

            foreach (Item i in itemsList) { //ensure all items that exist are listed as 'unmatched' by this current reassessment
                i.util = false;
            }

            for (int y = 0; y < height; y++) { //for every row
                for (int x = 0; x < width; x++) {
                    if (stateArray[x, y] == vp) { //check if it's a point that's been visited but not assigned, which shouldn't be the case
                        //Console.WriteLine(x + "," + y + " is Visited for some reason");
                    }

                    if (stateArray[x, y] == uvp) { //something to investigate!
                        itemSet.Clear(); //ensure things are clear
                        ptQ.Clear();

                        ptQ.Add(new Coord(x, y)); //add that position to the queue

                        while (ptQ.Count > 0) { //while there's something in ptQ
                            cord = ((Coord)ptQ[0]).clone(); //get the first thing in the queue out of it
                            stateArray[cord.x, cord.y] = vp; //label that point as visited
                            itemSet.Add(new Coord(cord.x, cord.y)); //add the coord to the itemSet
                            ptQ.RemoveAt(0); //get it the hell out of that queue!

                            //now for adjacent points!
                            //top
                            if (cord.y - 1 >= 0) { //if point is inside array bounds
                                if (stateArray[cord.x, cord.y - 1] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x, cord.y - 1)); //then add to the ptQ
                                    stateArray[cord.x, cord.y-1] = vp; //label that point as visited
                                }
                            }
                            //right
                            if (cord.x + 1 < width) { //if point is inside array bounds
                                if (stateArray[cord.x + 1, cord.y] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x + 1, cord.y)); //then add to the ptQ
                                    stateArray[cord.x+1, cord.y] = vp; //label that point as visited
                                }
                            }
                            //down
                            if (cord.y + 1 < height) { //if point is inside array bounds
                                if (stateArray[cord.x, cord.y + 1] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x, cord.y + 1)); //then add to the ptQ
                                    stateArray[cord.x, cord.y+1] = vp; //label that point as visited
                                }
                            }
                            //left
                            if (cord.x - 1 >= 0) { //if point is inside array bounds
                                if (stateArray[cord.x - 1, cord.y] == uvp) { //and point is a potential item point
                                    ptQ.Add(new Coord(cord.x - 1, cord.y)); //then add to the ptQ
                                    stateArray[cord.x-1, cord.y] = vp; //label that point as visited
                                }
                            }//end adj pt determination
                        }//end BFS while loop, itemSet contains entire set of points

                        if (itemSet.Count >= pointMin) { //we've got enough points to make an item - except for reassessment just check against current list
                            bool found = false;
                            float nearestErr = float.PositiveInfinity;
                            //Console.WriteLine("Rediscovered an item, determining...");
                            vol = calcVolumeFast((Coord[])itemSet.ToArray(typeof(Coord))); //calculate volume of object
                            cent = calcCentroid((Coord[])itemSet.ToArray(typeof(Coord))); //calculate centroid of object
                            //itemsList.Add(new Item(itemSet.Count, vol, cent[0], cent[1]));

                            //create temporary item
                            it = new Item(itemSet.Count, vol, cent[0], cent[1]);
                            //now compare
                            //note: maybe collect entire set of items first, then compare both sets, optimising minimum errors to get optimal matches on smaller items
                            foreach (Item i in itemsList) { //loop through confirmed items
                                
                                float iCompErr = i.compare(it); //compare the item found with the one from the arraylist
                                if (iCompErr <= comparisonError && i.util == false && found == false) { //see if one exists that matches close enough that hasnt been matched yet.
                                    
                                    Console.WriteLine("Item " + it.getString() + " rematched to " + i.getString() + " at " +iCompErr+" error.");
                                    for (int j = 0; j < itemSet.Count; j++) { //label all the points as item points (3)
                                        stateArray[((Coord)itemSet[j]).x, ((Coord)itemSet[j]).y] = ip; //set points to 3s
                                    }

                                    i.setXY(cent[0], cent[1]); //update that item's centroid
                                    i.util = true; //say that item's matched now
                                    found = true;
                                } else if (iCompErr < nearestErr) {
                                    nearestErr = iCompErr; //diagnostic purposes
                                }
                            }

                            if (found == false) { //however, if all items are checked and nothing is matched, then we can say its some new item or a limb or something and ignore it.
                                for (int i = 0; i < itemSet.Count; i++) { //label all the points as outlyers
                                    stateArray[((Coord)itemSet[i]).x, ((Coord)itemSet[i]).y] = op; //set points to 4s
                                }
                                Console.WriteLine("Item " + it.getString() + " not rematched, closest match at " + nearestErr + " error.");
                            }

                            //for (int i = 0; i < itemSet.Count; i++) { //label all the points as item points (3)
                            //    stateArray[((Coord)itemSet[i]).x, ((Coord)itemSet[i]).y] = ip; //set points to 3s
                            //}

                            //Console.WriteLine("Item rematched: " + itemSet.Count + ", " + vol + ", " + cent[0] + ", " + cent[1]);
                            //textBox1.AppendText("\nItem rematched: " + itemSet.Count + ", " + vol + ", " + cent[0] + ", " + cent[1]);
                            itemSet.Clear(); //clear out the itemset
                        } else { //otherwise there arent enough points, so flag those found as outlyers (4)
                            for (int i = 0; i < itemSet.Count; i++) { //label all the points as outlyers
                                stateArray[((Coord)itemSet[i]).x, ((Coord)itemSet[i]).y] = op; //set points to 4s
                            }
                        } //end classifying not points

                    }//end if we've found a point to BFS


                } //end x loop
            } //end y loop

            //how long did it take?
            stopwatch.Stop();
            Console.WriteLine("Time elapsed: " + stopwatch.Elapsed);

            //what state are the points in now?
            byte[] points = drawFieldInit(stateArray);
            int stride = width * 4;
            image1.Source = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, points, stride);

            //all adjacent points, if sufficient #, become new item by performing summations of points
            //foreach (Item i in itemsList) {
            //    Console.WriteLine(i.getString());
            //}
        }//end reIdItems


        /**
         * Initialise the game environment variables.
         */
        void gameEnvInit() {
            //int[,] hpArray = new int[width, height];
            //int[,] dmgArray = new int[width, height]; //each cycle overlay this on top of hpArray to deal damage.
            //bool cycleLooping = false; //meaning the game is running
            //bool envSet = false;
            //bool getGameFrame = false; //use this to grab a frame
            //int maxDamage = 30;
            //int healRate = 3;
            //Stopwatch gameTimer = new Stopwatch();

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (x > planeBoundsBFS[y, 0] && x < planeBoundsBFS[y, 1]) { //only fill in the areas that are 
                        hpArray[x, y] = 255;
                    }
                }
            }

            byte[] points = drawGame(hpArray);
            int stride = width * 4;
            image2.Source = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, points, stride);
            breach = false;
            gameTimer.Reset();
            cycleTimer.Reset();
            envSet = true;
            timeLeft = timeMax;
            label2.Content = timeLeft;
            Console.WriteLine("Game environment set up.");
        }//end gameEnvInit

        void gameStart() {
            gameTimer.Start();
            cycleTimer.Start();
            tickTimer.Start();
            cycleLooping = true;
            breach = false;
            getGameFrame = true;

            Console.WriteLine("Game started.");
        }

        //Stop game from cycling and reset all variables
        void gameStop() {
            getGameFrame = false;
            cycleLooping = false;
            gameTimer.Stop();
            gameTimer.Reset();
            cycleTimer.Stop();
            cycleTimer.Reset();
            tickTimer.Stop();
            tickTimer.Reset();
            breach = false;
            envSet = false;
            Console.WriteLine("Game stopped.");
        }

        /**
         * Main thing to call every interval to do what the game should.
         * Take item details and deal damage around them.
         * Also check if ice is breached.
         * Restrict to inside the bounds.
         */
        void gameCycle() {
            //int[,] hpArray = new int[width, height];
            //int[,] dmgArray = new int[width, height]; //each cycle overlay this on top of hpArray to deal damage.
            //bool cycleLooping = false; //meaning the game is running
            //bool envSet = false;
            //bool getGameFrame = false; //use this to grab a frame
            //bool breach = false; //game ends if true
            //int maxDamage = 26;
            //int dmgBonus = 4; //extra thing to add after calculation to ensure damage always gets dealt within radius
            //int healRate = 3;
            //Stopwatch gameTimer = new Stopwatch(); //every 3 or 5 sec set getGameFrame to true
            //itemsList
            int damage = 0;
            int dmgRadius = 0;
            int dist = 0;
            float distRatio = 0.0f;

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    dmgArray[x,y] = 0; //reset damage array, probably faster than asking for another 240x320 int array
                }
            }

            foreach (Item i in itemsList) {
                dmgRadius = (int)(Math.Sqrt((double)i.area) * 1.5); //dmgradius = sqrt(area) *1.5
                Console.WriteLine("Applying damage from " + i.getString() + " at radius: " + dmgRadius);
                for (int y = 0; y < height; y++) {
                    for (int x = 0; x < width; x++) {
                        if (x > planeBoundsBFS[y, 0] && x < planeBoundsBFS[y, 1]) { //if point in question is inside bounds
                            dist = (int)Math.Sqrt(Math.Pow(i.x - x, 2) + Math.Pow(i.y - y, 2)); //see how far it is from the centre of the object
                            if (dist <= dmgRadius) { //if within damaging radius
                                distRatio = 1- ((float)dist / (float)dmgRadius); //give it a falloff value
                                damage = (int)((maxDamage * distRatio * i.relWeight) + dmgBonus); //calculate damage to be done to that point
                                dmgArray[x, y] += damage; //apply damage to array, stacks for nearby items hence +=
                            }
                            
                        }//end if in plane bounds
                    }//end x
                }//end y

            }//end foreach item

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (x > planeBoundsBFS[y, 0] && x < planeBoundsBFS[y, 1]) { //if point in question is inside bounds
                        if (hpArray[x, y] < 255 - healRate) {
                            hpArray[x, y] += healRate; //remember to apply healing to each point
                        } else {
                            hpArray[x, y] = 255;
                        }
                    
                        hpArray[x, y] -= dmgArray[x, y]; //apply damage to hp
                        if (hpArray[x, y] <= 0) { //has to be less than 0 
                            breach = true; //o noes ice is broken! <------ Game loop should immediately stop, warnings and flashy things shoot everywhere, etc!
                            Console.WriteLine("Breach at: " + x + "," + y);
                        }
                    }
                }
            }

            


            byte[] points = drawGame(hpArray);
            int stride = width * 4;
            image2.Source = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, points, stride);
            //Console.WriteLine("Math fn test: " + Math.Sqrt(144) + ", " + Math.Pow(3, 2));
            if (breach) { //if game's ended then
                Console.WriteLine("Ice broken! Try again!"); //failure
                //gameTimer.Stop();
                //cycleTimer.Stop();
                //cycleLooping = false;
                //getGameFrame = false;
                gameStop();
            } else { //nevermind!
                Console.WriteLine("Safe for this cycle!");
            }

            //print2DArray(dmgArray);
        }//end gamecycle



        /**
         * Draw the game field into image2
         */
        byte[] drawGame(int[,] hpArr) {
            Byte[] points = new byte[width * height * 4]; //1 byte per colour per point
            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;
            const int AlphaIndex = 3;
            byte bval = 0;
            byte rval = 0;

            int colourIndex = 0;//gotta put this on the outside otherwise it colours the same line over and over >.>
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width && colourIndex < points.Length; x++, colourIndex += 4) { //writing into byte array too, so colour index
                    //if (hpArr[x, y] == -1) { //out of bounds pt - black
                    //    points[colourIndex + BlueIndex] = 0;
                    //    points[colourIndex + GreenIndex] = 0;
                    //    points[colourIndex + RedIndex] = 0;
                    //} 
                    if (x > planeBoundsBFS[y, 0] && x < planeBoundsBFS[y, 1]) {
                        if (hpArr[x, y] <= 255 && hpArr[x, y] >= 0) { //avoid overflow for byte type
                            bval = (byte)hpArr[x, y];
                            rval = (byte)(255 - hpArr[x, y]);
                        } else if (hpArr[x, y] < 0) {
                            bval = 0;
                            rval = 255;
                        } else {
                            bval = 255;
                            rval = 0;
                        }
                        points[colourIndex + BlueIndex] = bval;
                        points[colourIndex + GreenIndex] = 0;
                        points[colourIndex + RedIndex] = rval;
                    } else {
                        points[colourIndex + BlueIndex] = 0;
                        points[colourIndex + GreenIndex] = 0;
                        points[colourIndex + RedIndex] = 0;
                    }

                }//end x
            }//end y
            Console.WriteLine("Field redrawn.");
            return points;
        }


        /**
         * Identify the items on the desk. WARNING: SLOW AS HELL, O(n^2) COMPLEXITY!
         */
        void identifyItems() {

            ArrayList refinedPoints = new ArrayList(); //only potential item points for this (ccords only)
            ArrayList accumulator = new ArrayList(); //add coord objects to this one
            ArrayList ptQueue = new ArrayList(); //for the bfs
            ArrayList itemSet = new ArrayList();
            float pointError = float.PositiveInfinity;
            SkeletonPoint spointinQ = new SkeletonPoint();
            Coord cord = new Coord();
            Coord ctest = new Coord();
            int index = -1;
            int unvisited = 0;

            

            int[] cent = new int[2];
            float iVol = 0.0f;

            Item it;
            Stopwatch stopwatch = new Stopwatch();
            //Timer tick;

            //planeBounds[height,2] = edges of desk
            //skPtArray[width/x, height/y] = set of skeleton points

            //so we have the set of points for some frame, not the frame that was used for ransac either
            //sidenote: figure out a way to get 2d array for this thing

            Console.WriteLine("-----Detecting items...-----");
            textBox1.AppendText("\n-----Detecting items...-----");
            stopwatch.Start();

            //get rid of all the points not inside the bounds of the desk, leaving us with the points that relate to the game
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (x > planeBounds[y, 0] && x < planeBounds[y, 1]) { //if within the bounds of the desk
                        spointinQ = skClone(skPtArray[x, y]);
                        //now to get rid of all the points, that are within the same error as ransac, of the plane/desk, leaving with only item points or outliers
                        if (spointinQ.Z > 0) { //in front of camera
                            pointError = pointPlaneDistSP(finalPlane, spointinQ);
                            if (pointError > g) { //4cm from plane (likely to be an item)
                                refinedPoints.Add(new Coord(x, y));

                            }
                        }
                    }//end if in bounds
                }//end x
            }//end y
            textBox1.AppendText("\nrefinedPoints.Count: " + refinedPoints.Count);
            Console.WriteLine("refinedPoints.Count: " + refinedPoints.Count);
            //take any point from this subset

            stopwatch.Stop();
            Console.WriteLine("Point to plane calculations: " + stopwatch.Elapsed);
            stopwatch.Start();

            cord = ((Coord)refinedPoints[0]).clone();
            //refinedPoints.RemoveAt(0);
            ((Coord)refinedPoints[0]).util = true;
            unvisited = refinedPoints.Count-2;

            spointinQ = skClone(skPtArray[cord.x, cord.y]);
            ptQueue.Add(cord);
            //perform BFS type thing on it
            while (ptQueue.Count > 0 /*|| refinedPoints.Count > 0*/) { //while there's stuff in the ptQueue and there are still points to test
                cord = ((Coord)ptQueue[0]).clone();
                ptQueue.RemoveAt(0); //take first off queue
                ctest = cord.clone();
                //get adjacent points if they exist
                //Console.Write(cord.getStr() + ";");
                //x +/- 1
                ctest.x = cord.x + 1;
                //index = refinedPoints.IndexOf(ctest);
                index = findCoord(refinedPoints, ctest); //linear time search :(
                if (index >= 0 && ((Coord)refinedPoints[index]).util == false) {
                    ptQueue.Add(((Coord)refinedPoints[index]).clone());
                    ((Coord)refinedPoints[index]).util = true;
                    unvisited--;
                }
                ctest.x = cord.x - 1;
                //index = refinedPoints.IndexOf(ctest);
                index = findCoord(refinedPoints, ctest);
                if (index >= 0 && ((Coord)refinedPoints[index]).util == false) {
                    ptQueue.Add(((Coord)refinedPoints[index]).clone());
                    ((Coord)refinedPoints[index]).util = true;
                    unvisited--;
                }
                ctest.x = cord.x;
                //y +/- 1
                ctest.y = cord.y + 1;
                //index = refinedPoints.IndexOf(ctest);
                index = findCoord(refinedPoints, ctest);
                if (index >= 0 && ((Coord)refinedPoints[index]).util == false) {
                    ptQueue.Add(((Coord)refinedPoints[index]).clone());
                    ((Coord)refinedPoints[index]).util = true;
                    unvisited--;
                }
                ctest.y = cord.y - 1;
                //index = refinedPoints.IndexOf(ctest);
                index = findCoord(refinedPoints, ctest);
                if (index >= 0 && ((Coord)refinedPoints[index]).util == false) {
                    ptQueue.Add(((Coord)refinedPoints[index]).clone());
                    ((Coord)refinedPoints[index]).util = true;
                    unvisited--;
                }

                itemSet.Add(cord.clone()); //add the point index to the set of points that constitute an item

                if (ptQueue.Count == 0 && unvisited > 0) { //if we've completely covered a contiguous set of points but there are still more to visit
                    //maybe we've got an item, so take all values with 'util == true' out and see if they form something useful
                    
                    if (itemSet.Count > pointMin) { //if there are more than 300 points
                        
                        foreach (Coord c in itemSet) {
                            accumulator.Add(skClone(skPtArray[c.x, c.y])); //for every coordinate index thing in itemSet, add the corresponding 3D coords to accumulator
                        }
                        //create a new Item and add it to the itemsList
                        iVol = calcVolume((SkeletonPoint[])accumulator.ToArray(typeof(SkeletonPoint)));
                        cent = calcCentroid((Coord[])itemSet.ToArray(typeof(Coord)));
                        itemsList.Add(new Item(itemSet.Count, iVol, cent[0], cent[1]));
                        Console.WriteLine("New Item found! " + itemSet.Count + ", " + iVol + ", " + cent[0] + ", " + cent[1]);
                        textBox1.AppendText("\nNew Item found: " + itemSet.Count + ", " + iVol + ", " + cent[0] + ", " + cent[1]);
                        accumulator.Clear();

                    } //end if we have big enough set for item
                    Console.WriteLine("itemSet.Count: " + itemSet.Count+ ", ptQueue.Count: "+ptQueue.Count);
                    itemSet.Clear();

                    //start new set
                    for (int i = 0; i < refinedPoints.Count && ptQueue.Count == 0; i++) {
                        if (((Coord)refinedPoints[i]).util == false) { //we have a new starting point!
                            ptQueue.Add(((Coord)refinedPoints[i]).clone());
                            ((Coord)refinedPoints[i]).util = true;
                        }
                    }//end new start point finder

                } //end if nothing in queue but unfinished

            } //end while

            stopwatch.Stop();
            Console.WriteLine("Time elapsed: " + stopwatch.Elapsed);

            //all adjacent points, if sufficient #, become new item by performing summations of points
            foreach (Item i in itemsList) {
                Console.WriteLine(i.getString());
            }
        } //end item identification

        


        /**
         * Write out the entire array for some set
         */
        void print2DArray(int[,] intarray) {
            for (int y = 0; y < height; y++) {
                Console.WriteLine();
                for (int x = 0; x < width; x++) {
                    Console.Write("{0,2:n0}",intarray[x, y]);
                }
            }
            Console.WriteLine();

        }//end print2Darray


        /**
         * Calculate the sum of the distance from point to plane for an array of skeletonpoints.
         */
        float calcVolume(SkeletonPoint[] itemPoints) {
            float result = 0.0f;

            //for every item in the array, perform point to plane on it vs the desk plane, then add to result
            for (int i = 0; i < itemPoints.Length; i++) {
                result += pointPlaneDistSP(finalPlane, itemPoints[i]);
            }
            return result;
        }

        /**
         * Calculate the sum of the distance from point to plane for a set of coordinates
         */
        float calcVolumeFast(Coord[] coords) {
            float result = 0.0f;

            for (int i = 0; i < coords.Length; i++) {
                result += pointPlaneDistSP(finalPlane, skPtArray[coords[i].x, coords[i].y]);
            }

            return result;
        }

        /**
         * Calculate the centroid of some set of coordinates.
         */
        int[] calcCentroid(Coord[] coords) {
            int[] centroid = new int[2];
            int x = 0;
            int y = 0;

            for (int i = 0; i < coords.Length; i++) {
                x += coords[i].x;
                y += coords[i].y;
            }
            x /= coords.Length;
            y /= coords.Length;

            centroid[0] = x;
            centroid[1] = y;
            return centroid;
        }

        /**
         * Find a given Coord object within an ArrayList.
         */
        int findCoord(ArrayList coordlist, Coord cd) {
            int index = -1;

            for (int i = 0; i < coordlist.Count; i++) {
                if (((Coord)coordlist[i]).Equals(cd)) {
                    index = i;
                    break;
                }
            }
            return index;
        }

        /**
         * Clone a SkeletonPoint.
         */
        SkeletonPoint skClone(SkeletonPoint pt) {
            SkeletonPoint cln = new SkeletonPoint();
            cln.X = pt.X;
            cln.Y = pt.Y;
            cln.Z = pt.Z;
            
            return cln;

        }


        //where old code goes to die
        void junkCodeMethod() {

            /* Was used for running diagnostics and seeing about how C# handled objects*/
            Item test = new Item(1, 4.5f, 2, 3);
            Item test2 = new Item();
            Item test3 = new Item();

            textBox1.AppendText("\n");
            textBox1.AppendText(test.getString());
            textBox1.AppendText(test2.getString());
            test2 = test; //this reassigns reference rather than duplicating object
            textBox1.AppendText("test2 = test\n");
            textBox1.AppendText(test.getString());
            textBox1.AppendText(test2.getString());
            test2.setArea(5);
            test.setXY(6, 7);
            textBox1.AppendText("test2.setArea(5), test.setXY(6,7)\n");
            textBox1.AppendText(test.getString());
            textBox1.AppendText(test2.getString());

            test3 = test.clone(); //clone the object
            test3.setArea(9);
            test3.setVolume(12.0f);
            textBox1.AppendText("test3 = test.clone(), test3.setArea(9), test3.setVolume(12.0f)\n");
            textBox1.AppendText(test.getString());
            textBox1.AppendText(test3.getString());

            SkeletonPoint stest = new SkeletonPoint();
            SkeletonPoint stest2 = new SkeletonPoint();
            SkeletonPoint stest3 = new SkeletonPoint();
            stest.X = 3.0f;
            stest.Y = 4.0f;
            stest.Z = 5.0f;
            stest2.X = stest.X;
            stest2.Y = stest.Y;
            stest2.Z = stest.Z;
            stest3 = skClone(stest);
            textBox1.AppendText("\n");
            textBox1.AppendText("st : " + stest.X + "," + stest.Y + "," + stest.Z + "\n");
            textBox1.AppendText("st2: " + stest2.X + "," + stest2.Y + "," + stest2.Z + "\n");
            textBox1.AppendText("st3: " + stest3.X + "," + stest3.Y + "," + stest3.Z + "\n");
            stest.X = 6.0f;
            stest2.Y = 7.0f;
            stest3.Z = 9.0f;
            textBox1.AppendText("st : " + stest.X + "," + stest.Y + "," + stest.Z + "\n");
            textBox1.AppendText("st2: " + stest2.X + "," + stest2.Y + "," + stest2.Z + "\n");
            textBox1.AppendText("st3: " + stest3.X + "," + stest3.Y + "," + stest3.Z + "\n");

            /*for (int i = 0; i < height; i++) {
                Console.WriteLine(i + ": <" + planeBounds[i, 0] + "," + planeBounds[i, 1] + ">");
            }*/

            foreach (Item i in itemsList) {
                Console.WriteLine(i.getString());
            }

            Coord cd = new Coord(5, 8);
            ArrayList cds = new ArrayList();
            cds.Add(cd.clone());
            int cdi = cds.IndexOf(cd);
            textBox1.AppendText("\nCoord Equals test: " + cdi);
            cdi = findCoord(cds, cd);
            textBox1.AppendText("\nCoord Equals test: " + cdi);
            /*end*/




        } //end junk code method
        

    }
}
