using System;
using System.Collections;
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

//using System.Windows.Media.Media3D;
using Microsoft.Kinect;

namespace PlaneColouring {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window {
        public MainWindow() {
            InitializeComponent();
        }

        static string filename = "C:\\Users\\User\\DepthValueSets\\BoxPointCloud.txt"; //full path required
        //static string filename = "BoxPointCloud.txt";

        Random rng = new Random();
        Item it = new Item();

        static int height = 240, width = 320; //height and width of the depth frames
        //asdfadsfas
        string[] lines;
        int linecount = 0;
        float[,] pointarray; //2d array
        string[] temp = new string[3]; //temp after splitting line
        char[] delimiter = {' '};
        bool planeFound = false;

        float t = 0.03f; //max error (distance from point to plane, should be metres, but i have no idea, trial and error time

        ArrayList finalConsensusSet = new ArrayList();

        int[,] planeBounds = new int[height, 2]; //store of values to search for items in.
        int[,] planeBoundsBFS = new int[height, 2];
        float[] finalPlane = { 0, 0, 0, 0 }; //var for final plane eqtn


        private void Window_Loaded(object sender, RoutedEventArgs e) {
          
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e) {
            
        }

        //load cloud file
        private void button1_Click(object sender, RoutedEventArgs e) {
            int i, j = 0;
            lines = System.IO.File.ReadAllLines(@filename);
            linecount = lines.Length;
            pointarray = new float[linecount,3]; 

            for (i = 0; i < linecount; i++) { //for every line in the file
                temp = lines[i].Split(delimiter, 3);
                for (j = 0; j < 3; j++) { //build a line x 3 array of spatial points
                    pointarray[i,j] = float.Parse(temp[j]); 
                }
            }
            textBox1.AppendText("File data loaded.\n");
            textBox1.AppendText("Lines: " + linecount);
            for (i = 0; i < 10; i++) {
                textBox1.AppendText("\n" + lines[i]);
            }
            textBox1.AppendText("\n-------------\n");
            ////

            ///
            textBox1.AppendText("Values: " + pointarray.Length + "\n");
            for (i = 0; i < 10; i++) {
                for (j = 0; j < 3; j++) {
                    textBox1.AppendText(pointarray[i, j] + " ");
                }
                textBox1.AppendText("\n");
            }
            Console.WriteLine("Point cloud loaded");

        }

        /**
         * Button that calls ransac
         */
        private void button2_Click(object sender, RoutedEventArgs e) {
            //set some if to ensure stuff's been loaded already maybe
            int i = 0,j = 0;
            

            //call some ransac method around here
            ransac();
        }

        /**
         * Button for finding bounds of the plane
         */
        private void button3_Click(object sender, RoutedEventArgs e) {
            if (planeFound) {
                edgeFinderLinear();
                //writeRansacPC();
                //edgeFinderBFS();
            } else {
                Console.WriteLine("Run RANSAC first");
            }

            for (int i = 0; i < height; i++) {
                Console.WriteLine(i + ": <" + planeBounds[i, 0] + "," + planeBounds[i, 1] + ">");
            }
        }


        /**
         * RANSAC algorithm implementation
         */
        void ransac() {
            //data = pointarray [x,y,z]
            //model = plane = float[4]
            //linecount = number of points
            int n = 3; //# points reqd to fit a plane

            int k = 1000; //# of iterations

            int d = 15000; //# of close data values req'd to assert that a plane fits data
            
            ArrayList bestConsensusSet = new ArrayList(); //they dont have types!?
            ArrayList consensusSet = new ArrayList();
            float bestError = float.PositiveInfinity; //start error at the max value possible
            float sumError = 0;
            float pointError = 0;
            float thisError = float.PositiveInfinity; //the error for the current iteration
            //float[] bestPlane = new float[4]; //bestmodel
            float[] bestPlane = { 0, 0, 0, 0 }; //bestmodel
            float[] maybePlane = {0,0,0,0}; //maybemodel
            float[] thisPlane = {0,0,0,0}; //thismodel, also the model fitted to all points in consensus set (whatever that means)
            float[] pointinQ = new float[3]; //damn I need to use this, might also need to set to [x,y,z,n] where n is the array index
            int[] maybeInliers  = new int[n]; //array of indices of possible points
            float[] point1 = { 0, 0, 0 }; //c# is weird
            float[] point2 = { 0, 0, 0 };
            float[] point3 = { 0, 0, 0 };
            float[] pointx = { 0, 0, 0 };

            //how odd, to have to do this.
            //pointinQ[0] = pointarray[1,0];
            //pointinQ[1] = pointarray[1, 1];
            //pointinQ[2] = pointarray[1, 2];

            //////////////////////// need to make into skeletonpoints? not if im pro at alternate solutions!
            //SkeletonPoint test = new SkeletonPoint();
            //bestConsensusSet.Add(test);
            //bestConsensusSet.Add(pointinQ);
            Console.WriteLine("\n-----Ransacing...-----");

            for (int i = 0; i < k; i++) { //for each iteration
                consensusSet.Clear();
                sumError = 0;
                //maybeInliers = n Random points //also this could just be a 1d array of index points, why copy when they already exist?
                maybeInliers[0] = rng.Next(12800, linecount-12800); //bound them a little bit just so its more likely on the damn thing
                //maybeInliers[1] = rng.Next(12800, 64000);
                maybeInliers[1] = rng.Next(12800, linecount - 12800);
                maybeInliers[2] = rng.Next(12800, linecount - 12800);
                while (maybeInliers[0] == maybeInliers[1]) { //ensure no duplicate points
                    maybeInliers[1] = rng.Next(12800, linecount - 12800);
                }
                while (maybeInliers[0] == maybeInliers[2] || maybeInliers[1] == maybeInliers[2]) {
                    maybeInliers[2] = rng.Next(12800, linecount - 12800);
                }

                point1[0] = pointarray[maybeInliers[0], 0];//maybe pt1
                point1[1] = pointarray[maybeInliers[0], 1];
                point1[2] = pointarray[maybeInliers[0], 2];
                point2[0] = pointarray[maybeInliers[1], 0];//maybe pt2
                point2[1] = pointarray[maybeInliers[1], 1];
                point2[2] = pointarray[maybeInliers[1], 2];
                point3[0] = pointarray[maybeInliers[2], 0];//maybe pt3
                point3[1] = pointarray[maybeInliers[2], 1];
                point3[2] = pointarray[maybeInliers[2], 2];

                

                //maybePlane = fitPlane(maybeInliers);
                maybePlane = fitPlane(point1, point2, point3);
                //consensusSet = maybeInliers
                consensusSet.Add(maybeInliers[0]);
                consensusSet.Add(maybeInliers[1]);
                consensusSet.Add(maybeInliers[2]);

                //Console.WriteLine("Iteration " + i + " test plane: " + maybePlane[0] + "x + " + maybePlane[1] + "y + " + maybePlane[2] + "z + " + maybePlane[3] + " = 0");
                //Console.WriteLine("Random points: " + maybeInliers[0] + ", " + maybeInliers[1] + ", " + maybeInliers[2] + ";");

                for (int j = 0; j < linecount && !(maybeInliers.Contains(j)); j++) { //for every point that we didnt select already
                    //how odd of c#
                    //anyway, for each damn point, build an individual 1d array to represent it
                    if (pointarray[j, 2] > 0) { //if the point is actually away from the camera
                        pointinQ[0] = pointarray[j, 0];
                        pointinQ[1] = pointarray[j, 1];
                        pointinQ[2] = pointarray[j, 2];

                        pointError = pointPlaneDist(maybePlane, pointinQ); //the error is the distance of the point to the plane



                        if (pointError < t) { //if point's distance to plane is less than the error value
                            //consensusSet.Add(pointinQ);
                            consensusSet.Add(j); //add the index of the point to the consensus set
                            sumError += pointError;
                        }
                    }
                }
                
                if (consensusSet.Count > d) { //so we have enough matching points
                    
                    
                    //thisPlane = model parameters fitted to all points in consensusSet 
                    //thisError = measure of how well thisPlane fits the points
                    //divide by number of points to normalise
                    thisError = sumError / consensusSet.Count; //probably unneeded but whatever
                    
                    //if (thisError < bestError) { //then we have a new best model for something 
                    if (consensusSet.Count > bestConsensusSet.Count) { //so just use a higher number of points as the new result


                        //bestPlane = thisPlane; <--just use maybeplane, its the same damn thing
                        //thisPlane.CopyTo(bestPlane, 0);
                        bestPlane[0] = maybePlane[0];
                        bestPlane[1] = maybePlane[1];
                        bestPlane[2] = maybePlane[2];
                        bestPlane[3] = maybePlane[3];

                        //print out a whole ton of diagnostic stuff
                        Console.WriteLine("Best current plane ("+i+"): " + bestPlane[0] + "x + " + bestPlane[1] + "y + " + bestPlane[2] + "z + " + bestPlane[3] + " = 0");
                        Console.WriteLine("Random point indices: " + maybeInliers[0] + ", " + maybeInliers[1] + ", " + maybeInliers[2] + ";" + "\nThe three coordinates");

                        Console.WriteLine(pointarray[maybeInliers[0], 0] + "," + pointarray[maybeInliers[0], 1] + "," + pointarray[maybeInliers[0], 2]);
                        Console.WriteLine(pointarray[maybeInliers[1], 0] + "," + pointarray[maybeInliers[1], 1] + "," + pointarray[maybeInliers[1], 2]);
                        Console.WriteLine(pointarray[maybeInliers[2], 0] + "," + pointarray[maybeInliers[2], 1] + "," + pointarray[maybeInliers[2], 2]);
                        //index % width gives X coord, int casting the index over width gives Y coord
                        Console.WriteLine("X/Y grid coords: " + maybeInliers[0]%width + ","+ (int)(maybeInliers[0]/width)+" - "
                            + maybeInliers[1]%width + ","+ (int)(maybeInliers[1]/width)+" - "
                                + maybeInliers[2]%width + ","+ (int)(maybeInliers[2]/width));
                        Console.WriteLine("Aligning points: " +consensusSet.Count);
                        Console.WriteLine("Error: " +thisError);
                        //bestConsensusSet = consensusSet;

                        bestConsensusSet.Clear(); //clear out previous best set
                        foreach (int s in consensusSet) {//copy consensus set to bestconsensusset
                            bestConsensusSet.Add(s);
                        }
                        planeFound = true;
                        bestError = thisError;
                    }//end error val check
                }//end if consensus size
            }//end of iteration loop
            //finalPlane = bestPlane;
            //bestPlane.CopyTo(finalPlane,0);
            finalPlane[0] = bestPlane[0];
            finalPlane[1] = bestPlane[1];
            finalPlane[2] = bestPlane[2];
            finalPlane[3] = bestPlane[3];
            

            Console.WriteLine("\nFinal Plane Equation: " + finalPlane[0] + "x + " + finalPlane[1] + "y + " + finalPlane[2] + "z + " + finalPlane[3] + " = 0");

            foreach (int c in bestConsensusSet) {//copy consensus set to bestconsensusset
                finalConsensusSet.Add(c);
            }
            bestConsensusSet.Clear(); //have final consensus set
            consensusSet.Clear();

            
        }//end ransac method

        /**
         * Fit a plane of form ax + by + cz +d = 0 given 3 xyz coords
         */
        float[] fitPlane(float[] pt1, float[] pt2, float[] pt3) {
            float[] plane = new float[4];            
            int a=0, b=1, c=2, d=3;
            //pt<123>[x,y,z:0,1,2]
           
            //calculate plane given points
            plane[a] = pt1[1]*(pt2[2] - pt3[2]) + pt2[1]*(pt3[2] - pt1[2]) + pt3[1]*(pt1[2] - pt2[2]);
            plane[b] = pt1[2]*(pt2[0] - pt3[0]) + pt2[2]*(pt3[0] - pt1[0]) + pt3[2]*(pt1[0] - pt2[0]);
            plane[c] = pt1[0]*(pt2[1] - pt3[1]) + pt2[0]*(pt3[1] - pt1[1]) + pt3[0]*(pt1[1] - pt2[1]);
            // invert for ... = d (ie comment out the -1)
            plane[d] = (-1) * (pt1[0]*(pt2[1]*pt3[2] - pt3[1]*pt2[2]) + pt2[0]*(pt3[1]*pt1[2] - pt1[1]*pt3[2]) + pt3[0]*(pt1[1]*pt2[2] - pt2[1]*pt1[2]));


            return plane;
        }//end fitPlane method

        /**
         * Find the smallest (ie perpendicular) distance from a given point to a given plane.
         */
        float pointPlaneDist(float[] plane, float[] point) {
            float result = 0;
            float topline = 0;
            float botline = 0;
            //D = |ax + by + cz + d| / sqrt(a^2 + b^2 + c^2)
            //magic
            //better ensure this value comes out as absolute
            topline = plane[0]*point[0] + plane[1]*point[1] + plane[2]*point[2] + plane[3];
            topline = Math.Abs(topline);
            botline = (float)(Math.Pow(plane[0], 2) + Math.Pow(plane[1], 2) + Math.Pow(plane[2], 2));
            botline = (float)Math.Sqrt(botline);

            if (botline != 0) {
                result = topline / botline;
            }
            return result;
        } //end pointPlaneDist method

        /**
         * Attempt to find the edges of the plane via linear scan of the point set.
         * May require realignment of points into 2d array.
         */ 
        void edgeFinderLinear() {
            int i = 0, j = 0;
            int offset;
            int coordX = 0, coordY = 0;
            float[] pointinQ = new float[3];
            bool leftbound = false, rightbound = false;
            float pointError = float.PositiveInfinity;
            //float t = 0.04f; //nearness a point has to be, redefined as global
            

            for (i = 0; i < height; i++) {//set all to initial values
                planeBounds[i, 0] = -1;
                planeBounds[i, 1] = -1;
            }
            //index % width gives X coord, int casting the index over width gives Y coord
            //Console.WriteLine("X/Y grid coords: " + maybeInliers[0] % width + "," + (int)(maybeInliers[0] / width) + " - "
             //   + maybeInliers[1] % width + "," + (int)(maybeInliers[1] / width) + " - "
              //      + maybeInliers[2] % width + "," + (int)(maybeInliers[2] / width));
            //0-319, 320-639, 0->n-1, n->2n-1 for each row
            //offset = i*320
            j = width; //the line limiter thingy
            offset = 0; //basically line counter
            for (i = 0; i < linecount; i++) { //find all the left side bounds
                
                //offset = i * 320;
                if (i >= j || leftbound == true) { //if we've reached the start of the next line
                    i = j; //move i up to start of next line
                    offset++; //increment line counter
                    j += width; //move line limiter up to next one

                    leftbound = false;
                }

                pointinQ[0] = pointarray[i, 0]; //get the point from the array of points we have already
                pointinQ[1] = pointarray[i, 1];
                pointinQ[2] = pointarray[i, 2];

                pointError = pointPlaneDist(finalPlane, pointinQ);

                if (pointError < t) {
                    planeBounds[offset, 0] = i % width; // offset gives y coord, i % width gives x coord
                    
                    leftbound = true;
                }
            } //end forward search

            j = linecount - (width +1);
            offset = height - 1;
            for (i = linecount - 1; i >= 0; i--) { //find all left side bounds
                //if (planeBounds[offset, 0] != -1) { //if there's a left bound on this line

                    if (i < j || rightbound == true) {
                        i = j;
                        offset--;
                        j -= width;
                        rightbound = false;
                    }

                    pointinQ[0] = pointarray[i, 0]; //get the point from the array of points we have already
                    pointinQ[1] = pointarray[i, 1];
                    pointinQ[2] = pointarray[i, 2];

                    pointError = pointPlaneDist(finalPlane, pointinQ);

                    if (pointError < t) {
                        planeBounds[offset, 1] = i % width; //offset gives y coord, i % width gives x coord

                        rightbound = true;
                    }


                //}
                /* else { //move up a line
                    i = j;
                    j -= width;
                    offset--;
                }*/
            } //end backward search

        }

        /**
         * Attempt to find plane edges with BFS from the points that I already know are on there.
         */
        void edgeFinderBFS() {
            ArrayList pointsOnPlane = new ArrayList();
            ArrayList ptQueue = new ArrayList(); //queue
            int i, curr;
            int startPoint = -1;
            bool[] visited = new bool[linecount];
            int[] adjPoints = { -1, -1, -1, -1 }; //top, right, bot, left (clockwise)

            float[] pointinQ = new float[3];
            bool leftbound = false, rightbound = false;
            float pointError = float.PositiveInfinity;


            for (i = 0; i < height; i++) {//set all to initial values
                planeBoundsBFS[i, 0] = -1;
                planeBoundsBFS[i, 1] = -1;
            }

            startPoint = (int)finalConsensusSet[0]; //the first randomly selected point (ie guaranteed to be on plane) is starting point of bfs

            ptQueue.Add(startPoint);

            while (ptQueue.Count > 0) { //while there are still points to search
                curr = (int)ptQueue[0]; //get the first point in the queue
                ptQueue.RemoveAt(0);
                visited[curr] = true;

                //if the point is on the plane

                pointinQ[0] = pointarray[curr, 0]; //get the point from the array of points we have already
                pointinQ[1] = pointarray[curr, 1];
                pointinQ[2] = pointarray[curr, 2];

                pointError = pointPlaneDist(finalPlane, pointinQ);

                adjPoints = adjPointFinder(curr); //get the adjacent points
                foreach (int k in adjPoints) { 
                    if (k >= 0 && visited[k] != true) { //if there actually is an adjacent point on this side
                        ptQueue.Add(k); //add to end of queue
                    }
                }

            } //end while


        }

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
            if ((pt % width) > 0 && (pt% width) < (width-1)) { //if the point isnt on the left or right edge of the line, then left/right = pt +/-1
                result[1] = width + 1; //right
                result[3] = width - 1; //left
            } else if ((pt % width) == 0) { //left side
                result[1] = width + 1; //right
            } else { //right side
                result[3] = width - 1; //left
            }

            return result;
        }


        

        /**
         * Runs a breadth first search on the graph from a specified source
         *
         * @param g the graph
         * @param source the source vertex
 
         void graph_bfs(graph g, int source) {
            int i;
            int u = 0;
            int v = 0;
            queue que = queue_new(g->capacity);
            for (i = 0; i < g->capacity; i++) {
                g->verts[i].state = UNVISITED;
                g->verts[i].dist = -1;
                g->verts[i].pred = -1;
            }
            g->verts[source].state = VISITED_SELF;
            g->verts[source].dist = 0;
            queue_add(que, source);
            while (queue_is_empty(que) != 1) {
                u = queue_remove(que);
                for (v = 0; v < g->capacity; v++) {
                    if ((g->verts[v].state == UNVISITED) && g->edges[u][v] == 1) {
                        g->verts[v].state = VISITED_SELF;
                        g->verts[v].dist = 1 + g->verts[u].dist;
                        g->verts[v].pred = u;
                        queue_add(que, v);
                    }
                }
                g->verts[u].state = VISITED_DESCENDANTS;
            }

            queue_delete(que);
        }
        
        */


        void writeRansacPC() {
            String filepath = "C:\\Users\\User\\"; //an output path
            //SkeletonPoint[] skPoints = new SkeletonPoint[depthFrame.PixelDataLength]; //so we 
            int j;
            int curr = 0;
            //short[] rawDepthD = new short[depthFrame.PixelDataLength];
            //depthFrame.CopyPixelDataTo(rawDepthD);


            
            System.IO.StreamWriter filePC = new System.IO.StreamWriter(filepath + "RansacPC.ply");

            filePC.WriteLine("ply");
            filePC.WriteLine("format ascii 1.0");
            filePC.WriteLine("element vertex 76800");
            filePC.WriteLine("property float x");
            filePC.WriteLine("property float y");
            filePC.WriteLine("property float z");
            filePC.WriteLine("property uchar red");
            filePC.WriteLine("property uchar green");
            filePC.WriteLine("property uchar blue");
            filePC.WriteLine("end_header");

            /*for (j = 0; j < rawDepthD.Length; j++) {
                fileDM.WriteLine(rawDepthD[j]); //write all depth values to a file
            }*/

            j = 0;
            for (j = 0; j < linecount; j++) {

                if (finalConsensusSet.Contains(j)) { //if the consensusset has the point in it, then it's a point on the plane
                    filePC.WriteLine(pointarray[j,0] + " "+ pointarray[j, 1] + " " + pointarray[j, 2] + " 255 0 0");
                } else {
                    filePC.WriteLine(pointarray[j,0] + " "+ pointarray[j, 1] + " " + pointarray[j, 2] + " 100 100 100");
                }

            }

            /*for (int y = 0; y < depthFrame.Height; y++) { //
                for (int x = 0; x < depthFrame.Width; x++) { //iterate through width then depth of the data, writing all points to another file in meters
                    skPoints[j] = depthFrame.MapToSkeletonPoint(x, y);
                    fileSP.Write(skPoints[j].X + " ");
                    fileSP.Write(skPoints[j].Y + " ");
                    fileSP.WriteLine(skPoints[j].Z);
                    j++;
                }
            }*/




            filePC.Close();
            
            textBox1.AppendText("\nFile written.");
        }
    }//end partial class mainwindow
}//end namespace planecolouring
