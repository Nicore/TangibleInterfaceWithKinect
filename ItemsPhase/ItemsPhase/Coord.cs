using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ItemsPhase {
    public class Coord {

        public int x;
        public int y;
        public bool util; //utility boolean -- epically poor practice

        public Coord() {
            this.x = 0;
            this.y = 0;
            this.util = false;
        }

        public Coord(int x, int y) {
            this.x = x;
            this.y = y;
            this.util = false;
        }

        public Coord clone() {
            Coord ret = new Coord(this.x, this.y);
            return ret;
        }

        public bool Equals(Coord pt) {
            bool ret = false;
            if (pt.x == this.x && pt.y == this.y) {
                ret = true;
            }
            return ret;
        }

        public String getStr() {
            return "x:" + x + ",y:" + y;
        }

    }
}
