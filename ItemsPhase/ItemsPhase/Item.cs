using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ItemsPhase {

    /**
     * Class to use for item maintenance
     */
    public class Item {

        public int area; //area of item in # of pixels
        public float volume; //volume of item done by numerical integration of all points
        public int x; //nearest absolute pixel coordinates
        public int y;
        public bool util = false;
        public float relWeight = 0.0f; //relative weighting for damage calculations, as a % of the maximum item's volume

        //default constructor
        public Item() {
            area = 0;
            volume = 0.0f;
            x = 0;
            y = 0;
        }

        /**
         * Constructor yay
         */
        public Item(int area, float volume, int x, int y) {
            this.area = area;
            this.volume = volume;
            this.x = x;
            this.y = y;
        }

        //mutators
        public void setArea(int area) {
            this.area = area;
        }

        public void setVolume(float vol) {
            volume = vol;
        }

        public void setXY(int x, int y) {
            this.x = x;
            this.y = y;
        }

        //accessors
        public int getArea() {
            return area;
        }

        public float getVolume() {
            return volume;
        }

        public int getX() {
            return x;
        }

        public int getY() {
            return y;
        }


        public String getString() {
            String ret;
            ret = "A="+this.area+", V="+this.volume+", x="+this.x+", y="+this.y+", W="+ this.relWeight+"";


            return ret;
        }

        public Item clone() {
            Item cloned = new Item(this.area, this.volume, this.x, this.y);
            return cloned;
        }

        /**
         * Compare given item with this instance. Return an error value where lower value indicates given item is most likely the same item.
         */
        public float compare(Item it) {
            float error = float.PositiveInfinity; //how much the given item differs from this instance
            int areaDiff = Math.Abs(this.area - it.getArea()); //absolute difference in number of points
            float volDiff = Math.Abs(this.volume - it.getVolume()); //absolute difference in volume
            float areaDiffRatio = areaDiff / this.area; //how different the item being tested's area differs from this one's.
            float volDiffRatio = volDiff / this.volume; //likewise for volume

            error = (areaDiffRatio + volDiffRatio) / 2; //naive and ridiculous method because I can't think of anything better


            return error;
        }

    }
}
