using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;

using Rhino;
using Rhino.Geometry;
using Rhino.DocObjects;
using Rhino.Collections;

using GH_IO;
using GH_IO.Serialization;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace Robot_Elements
{
    public class RPlane
    {
        public Plane P;

        public RPlane()
        {
            P = new Plane();
        }

        public RPlane(Plane Pinput)
        {
            P = Pinput;
        }

        public Matrix<double> Plane2Matrix()
        {
            Matrix<double> M = Matrix<double>.Build.Dense(4, 4, 0);

            M[0, 0] = P.XAxis.X; M[0, 1] = P.YAxis.X; M[0, 2] = P.ZAxis.X; M[0, 3] = P.OriginX;
            M[1, 0] = P.XAxis.Y; M[1, 1] = P.YAxis.Y; M[1, 2] = P.ZAxis.Y; M[1, 3] = P.OriginY;
            M[2, 0] = P.XAxis.Z; M[2, 1] = P.YAxis.Z; M[2, 2] = P.ZAxis.Z; M[2, 3] = P.OriginZ;
            M[3, 3] = 1;

            return M;
        }

        public Transform PlanetoTransform()
        {
            Transform M = new Transform();

            M[0, 0] = P.XAxis.X; M[0, 1] = P.YAxis.X; M[0, 2] = P.ZAxis.X; M[0, 3] = P.OriginX;
            M[1, 0] = P.XAxis.Y; M[1, 1] = P.YAxis.Y; M[1, 2] = P.ZAxis.Y; M[1, 3] = P.OriginY;
            M[2, 0] = P.XAxis.Z; M[2, 1] = P.YAxis.Z; M[2, 2] = P.ZAxis.Z; M[2, 3] = P.OriginZ;
            M[3, 3] = 1;

            return M;
        }

    }

    public class Robot_IK
    {
        public double[] A;
        public double[] D;

        public struct t_Limit
        {
            public double Upper;
            public double Lower;
        }
        public t_Limit[] Joint_Limits;

        public struct t_Position
        {
            public double X;
            public double Y;
            public double Z;
        };

        public Point3d Position;

        public struct t_Orientation
        {
            public Vector3d XAxis;
            public Vector3d YAxis;
            public Vector3d ZAxis;
        }

        public t_Orientation Orientation;

        public Plane EOT_Pose;

        public Matrix<double> JointAngles;
        public Matrix<double> Je;
        public Matrix<double> J1_pose;
        public Matrix<double> J2_pose;
        public Matrix<double> J3_pose;
        public Matrix<double> J4_pose;
        public Matrix<double> J5_pose;
        public Matrix<double> J6_pose;

        public struct t_IK_Chain
        {
            // DH-transform matrices
            public Matrix<double> TM1;
            public Matrix<double> TM2;
            public Matrix<double> TM3;
            public Matrix<double> TM4;
            public Matrix<double> TM5;
            public Matrix<double> TM6;
            public Matrix<double> TM;

            // Derivatives 
            public Matrix<double> dTM1;
            public Matrix<double> dTM2;
            public Matrix<double> dTM3;
            public Matrix<double> dTM4;
            public Matrix<double> dTM5;
            public Matrix<double> dTM6;

        }

        public t_IK_Chain IK_Chain;

        public Plane ActualBasePlane;

        public Robot_IK()
        {
            A = new double[6];
            D = new double[6];

            Joint_Limits = new t_Limit[6];

            IK_Chain.TM = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.TM1 = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.TM2 = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.TM3 = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.TM4 = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.TM5 = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.TM6 = Matrix<double>.Build.Dense(4, 4);

            IK_Chain.dTM1 = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.dTM2 = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.dTM3 = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.dTM4 = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.dTM5 = Matrix<double>.Build.Dense(4, 4);
            IK_Chain.dTM6 = Matrix<double>.Build.Dense(4, 4);

            JointAngles = Matrix<double>.Build.Dense(6, 1, 0);
            Je = Matrix<double>.Build.Dense(12, 6, 0);

            this.IK_Chain.TM1[2, 1] = -1;
            this.IK_Chain.TM1[3, 3] = 1;

            this.IK_Chain.TM2[2, 2] = -1;
            this.IK_Chain.TM2[3, 3] = 1;

            this.IK_Chain.TM3[2, 1] = -1;
            this.IK_Chain.TM3[3, 3] = 1;

            this.IK_Chain.TM4[2, 1] = 1;
            this.IK_Chain.TM4[3, 3] = 1;

            this.IK_Chain.TM5[2, 1] = -1;
            this.IK_Chain.TM5[3, 3] = 1;

            this.IK_Chain.TM6[2, 2] = -1;
            this.IK_Chain.TM6[3, 3] = 1;

            JointAngles[1, 0] = -Math.PI / 2;

            Update_Pose();
        }

        private void Update_IKChain()
        {
            double c;
            double s;

            // Update DH transform matrix TM1
            //
            // TM1 = [c1   0  -s1  A1*c1]        dTM1 = [-s1   0  -c1  -A1*s1]
            //       [s1   0   c1  A1*s1]               [ c1   0  -s1   A1*c1]
            //       [ 0  -1    0     D1]               [  0   0    0       0]
            //       [ 0   0    0      1]               [  0   0    0       0]
            //

            // Compute c1 = cos(Joint angle 1)
            //         s1 = sin(Joint angle 1)
            c = Math.Cos(JointAngles[0, 0]);
            s = Math.Sin(JointAngles[0, 0]);

            IK_Chain.TM1[0, 0] = c;
            IK_Chain.TM1[0, 2] = -1 * s;
            IK_Chain.TM1[0, 3] = A[0] * c;

            IK_Chain.TM1[1, 0] = s;
            IK_Chain.TM1[1, 2] = c;
            IK_Chain.TM1[1, 3] = A[0] * s;

            IK_Chain.dTM1[0, 0] = -1 * s;
            IK_Chain.dTM1[0, 2] = -1 * c;
            IK_Chain.dTM1[0, 3] = -A[0] * s;

            IK_Chain.dTM1[1, 0] = c;
            IK_Chain.dTM1[1, 2] = -1 * s;
            IK_Chain.dTM1[1, 3] = A[0] * c;

            // Update DH transform matrix TM2
            //
            // TM2 = [c2  s2    0  A2*c2]        dTM2 = [-s2  c2   0  -A2*s2]
            //       [s2 -c2    0  A2*s2]               [ c2  s2   0   A2*c2]
            //       [ 0   0   -1      0]               [  0   0   0       0]
            //       [ 0   0    0      1]               [  0   0   0       0]
            //
            c = Math.Cos(JointAngles[1, 0]);
            s = Math.Sin(JointAngles[1, 0]);

            IK_Chain.TM2[0, 0] = c;
            IK_Chain.TM2[0, 1] = s;
            IK_Chain.TM2[0, 3] = A[1] * c;

            IK_Chain.TM2[1, 0] = s;
            IK_Chain.TM2[1, 1] = -1 * c;
            IK_Chain.TM2[1, 3] = A[1] * s;

            IK_Chain.dTM2[0, 0] = -1 * s;
            IK_Chain.dTM2[0, 1] = c;
            IK_Chain.dTM2[0, 3] = -A[1] * s;

            IK_Chain.dTM2[1, 0] = c;
            IK_Chain.dTM2[1, 1] = s;
            IK_Chain.dTM2[1, 3] = A[1] * c;

            // Update DH transform matrix TM3
            //
            // TM3 = [c3   0  -s3  A3*c3]        dTM3 = [-s3  0  -c3  -A3*s3]
            //       [s3   0   c3  A3*s3]               [ c3  0  -s3   A3*c3]
            //       [ 0  -1    0      0]               [  0  0    0       0]
            //       [ 0   0    0      1]               [  0  0    0       0]
            //
            c = Math.Cos(JointAngles[2, 0]);
            s = Math.Sin(JointAngles[2, 0]);

            IK_Chain.TM3[0, 0] = c;
            IK_Chain.TM3[0, 2] = -1 * s;
            IK_Chain.TM3[0, 3] = A[2] * c;

            IK_Chain.TM3[1, 0] = s;
            IK_Chain.TM3[1, 2] = c;
            IK_Chain.TM3[1, 3] = A[2] * s;

            IK_Chain.dTM3[0, 0] = -1 * s;
            IK_Chain.dTM3[0, 2] = -1 * c;
            IK_Chain.dTM3[0, 3] = -A[2] * s;

            IK_Chain.dTM3[1, 0] = c;
            IK_Chain.dTM3[1, 2] = -1 * s;
            IK_Chain.dTM3[1, 3] = A[2] * c;

            // Update DH transform matrix TM4
            //
            // TM4 = [c4   0   s4      0]        dTM4 = [-s4   0   c4   0]
            //       [s4   0  -c4      0]               [ c4   0   s4   0] 
            //       [ 0   1    0    -D4]               [  0   0    0   0]
            //       [ 0   0    0      1]               [  0   0    0   0]
            //
            c = Math.Cos(JointAngles[3, 0]);
            s = Math.Sin(JointAngles[3, 0]);

            IK_Chain.TM4[0, 0] = c;
            IK_Chain.TM4[0, 2] = s;

            IK_Chain.TM4[1, 0] = s;
            IK_Chain.TM4[1, 2] = -c;

            IK_Chain.dTM4[0, 0] = -1 * s;
            IK_Chain.dTM4[0, 2] = c;

            IK_Chain.dTM4[1, 0] = c;
            IK_Chain.dTM4[1, 2] = s;

            // Update DH transform matrix TM5
            //
            // TM5 = [c5   0  -s5      0]        dTM5 = [-s5   0  -c5   0]  
            //       [s5   0   c5      0]               [ c5   0  -s5   0]
            //       [ 0  -1    0      0]               [  0   0    0   0]
            //       [ 0   0    0      1]               [  0   0    0   0]
            //
            c = Math.Cos(JointAngles[4, 0]);
            s = Math.Sin(JointAngles[4, 0]);

            IK_Chain.TM5[0, 0] = c;
            IK_Chain.TM5[0, 2] = -1 * s;

            IK_Chain.TM5[1, 0] = s;
            IK_Chain.TM5[1, 2] = c;

            IK_Chain.dTM5[0, 0] = -1 * s;
            IK_Chain.dTM5[0, 2] = -1 * c;

            IK_Chain.dTM5[1, 0] = c;
            IK_Chain.dTM5[1, 2] = -1 * s;

            // Update DH transform matrix TM6
            //
            // TM6 = [c6  -s6    0      0]        dTM6 = [-s6   c6   0   0]
            //       [s6 c6    0      0]               [ c6   s6   0   0]
            //       [ 0   0   1      0]               [  0    0   0   0]
            //       [ 0   0    0      1]               [  0    0   0   0]
            //
            c = Math.Cos(JointAngles[5, 0]);
            s = Math.Sin(JointAngles[5, 0]);

            IK_Chain.TM6[0, 0] = c;
            IK_Chain.TM6[0, 1] = s;            

            IK_Chain.TM6[1, 0] = s;
            IK_Chain.TM6[1, 1] = -c;

            IK_Chain.dTM6[0, 0] = -1 * s;
            IK_Chain.dTM6[0, 1] = c;

            IK_Chain.dTM6[1, 0] = c;
            IK_Chain.dTM6[1, 1] = s;

        }

        public void Update_Pose()
        {
            Update_IKChain();
            /*
            IK_Chain.TM2 = IK_Chain.TM1 * IK_Chain.TM2;
            IK_Chain.TM3 = IK_Chain.TM2 * IK_Chain.TM3;
            IK_Chain.TM4 = IK_Chain.TM3 * IK_Chain.TM4;
            IK_Chain.TM5 = IK_Chain.TM4 * IK_Chain.TM5;
            IK_Chain.TM6 = IK_Chain.TM5 * IK_Chain.TM6;

            IK_Chain.TM = IK_Chain.TM6;
            */
            J1_pose = IK_Chain.TM1;
            J2_pose = J1_pose * IK_Chain.TM2;
            J3_pose = J2_pose * IK_Chain.TM3;
            J4_pose = J3_pose * IK_Chain.TM4;
            J5_pose = J4_pose * IK_Chain.TM5;
            J6_pose = J5_pose * IK_Chain.TM6;

            IK_Chain.TM = J6_pose;

            Position.X = IK_Chain.TM[0, 3];
            Position.Y = IK_Chain.TM[1, 3];
            Position.Z = IK_Chain.TM[2, 3];

            Orientation.XAxis.X = IK_Chain.TM[0, 0];
            Orientation.XAxis.Y = IK_Chain.TM[1, 0];
            Orientation.XAxis.Z = IK_Chain.TM[2, 0];

            Orientation.YAxis.X = IK_Chain.TM[0, 1];
            Orientation.YAxis.Y = IK_Chain.TM[1, 1];
            Orientation.YAxis.Z = IK_Chain.TM[2, 1];

            Orientation.ZAxis.X = IK_Chain.TM[0, 2];
            Orientation.ZAxis.Y = IK_Chain.TM[1, 2];
            Orientation.ZAxis.Z = IK_Chain.TM[2, 2];
            
        }

        public void Update_Je()
        {
            Matrix<double> Temp;

            Temp = IK_Chain.dTM1 * IK_Chain.TM2 * IK_Chain.TM3 * IK_Chain.TM4 * IK_Chain.TM5 * IK_Chain.TM6;

            Je.SetColumn(0, Columnize(Temp));

            Temp = IK_Chain.TM1 * IK_Chain.dTM2 * IK_Chain.TM3 * IK_Chain.TM4 * IK_Chain.TM5 * IK_Chain.TM6;

            Je.SetColumn(1, Columnize(Temp));

            Temp = IK_Chain.TM1 * IK_Chain.TM2 * IK_Chain.dTM3 * IK_Chain.TM4 * IK_Chain.TM5 * IK_Chain.TM6;

            Je.SetColumn(2, Columnize(Temp));

            Temp = IK_Chain.TM1 * IK_Chain.TM2 * IK_Chain.TM3 * IK_Chain.dTM4 * IK_Chain.TM5 * IK_Chain.TM6;

            Je.SetColumn(3, Columnize(Temp));

            Temp = IK_Chain.TM1 * IK_Chain.TM2 * IK_Chain.TM3 * IK_Chain.TM4 * IK_Chain.dTM5 * IK_Chain.TM6;

            Je.SetColumn(4, Columnize(Temp));

            Temp = IK_Chain.TM1 * IK_Chain.TM2 * IK_Chain.TM3 * IK_Chain.TM4 * IK_Chain.TM5 * IK_Chain.dTM6;
            
            Je.SetColumn(5, Columnize(Temp));

        }

        public Vector<double> Columnize(Matrix<double> M)
        {
            Vector<double> C = Vector<double>.Build.Dense(M.ColumnCount * (M.RowCount - 1));

            int i;
            int j;
            int k;
            
            for (k = 0, j = 0; j < M.ColumnCount; j++)
                for (i = 0; i < M.RowCount - 1; i++)
                {
                    C[k++] = M[i, j];
                }

            return C;
        }
        
        //
        // Returns the orientation and origin data from a plane as a column matrix (of size 12).
        // C represnts the pose in IK calculations.
        //
        public Matrix<double> ColumnizePlane(Plane P)
        {
            Matrix<double> C = Matrix<double>.Build.Dense(12, 1);

            C[0, 0] = P.XAxis.X; C[1, 0] = P.XAxis.Y; C[2, 0] = P.XAxis.Z;
            C[3, 0] = P.YAxis.X; C[4, 0] = P.YAxis.Y; C[5, 0] = P.YAxis.Z;
            C[6, 0] = P.ZAxis.X; C[7, 0] = P.ZAxis.Y; C[8, 0] = P.ZAxis.Z;
            C[9, 0] = P.OriginX; C[10, 0] = P.OriginY; C[11, 0] = P.OriginZ;

            return C;
        }
        
        //
        // Returns the position and orientation of the EOT as a column  matrix (of size 12).
        //
        public Matrix<double> GetPose()
        {
            Matrix<double> pose = Matrix<double>.Build.Dense(12, 1);

            pose[0, 0] = this.Orientation.XAxis.X; pose[1, 0] = this.Orientation.XAxis.Y; pose[2, 0] = this.Orientation.XAxis.Z;
            pose[3, 0] = this.Orientation.YAxis.X; pose[4, 0] = this.Orientation.YAxis.Y; pose[5, 0] = this.Orientation.YAxis.Z;
            pose[6, 0] = this.Orientation.ZAxis.X; pose[7, 0] = this.Orientation.ZAxis.Y; pose[8, 0] = this.Orientation.ZAxis.Z;
            pose[9, 0] = this.Position.X; pose[10, 0] = this.Position.Y; pose[11, 0] = this.Position.Z;

            return pose;
        }

        public Matrix<double> Plane2Matrix(Plane P)
        {
            Matrix<double> M = Matrix<double>.Build.Dense(4, 4, 0);

            M[0, 0] = P.XAxis.X; M[0, 1] = P.YAxis.X; M[0, 2] = P.ZAxis.X; M[0, 3] = P.OriginX;
            M[1, 0] = P.XAxis.Y; M[1, 1] = P.YAxis.Y; M[1, 2] = P.ZAxis.Y; M[1, 3] = P.OriginY;
            M[2, 0] = P.XAxis.Z; M[2, 1] = P.YAxis.Z; M[2, 2] = P.ZAxis.Z; M[2, 3] = P.OriginZ;
            M[3, 0] = 0; M[3, 1] = 0; M[3, 2] = 0; M[3, 3] = 1;

            return M;
        }

        public Matrix<double> ColumnizeMatrix(Matrix<double> M)
        {
            Matrix<double> P = Matrix<double>.Build.Dense(12, 1);

            P[0, 0] = M[0, 0]; P[1, 0] = M[1, 0]; P[2, 0] = M[2, 0];
            P[3, 0] = M[0, 1]; P[4, 0] = M[1, 1]; P[5, 0] = M[2, 1];
            P[6, 0] = M[0, 2]; P[7, 0] = M[1, 2]; P[8, 0] = M[2, 2];
            P[9, 0] = M[0, 3]; P[10, 0] = M[1, 3]; P[11, 0] = M[2, 3];

            return P;
        }

        public Plane Matrix2Plane(Matrix<double> M)
        {
            Plane P = new Plane();

            P.Origin = new Point3d(M[0,3], M[1,3], M[2,3]);
            P.XAxis = new Vector3d(M[0, 0], M[1, 0], M[2, 0]);
            P.YAxis = new Vector3d(M[0, 1], M[1, 1], M[2, 1]);
            P.ZAxis = new Vector3d(M[0, 2], M[1, 2], M[2, 2]);

            return P;
        }

        public Transform PlanetoTransform(Plane P)
        {
            Transform M = new Transform();

            M[0, 0] = P.XAxis.X; M[0, 1] = P.YAxis.X; M[0, 2] = P.ZAxis.X; M[0, 3] = P.OriginX;
            M[1, 0] = P.XAxis.Y; M[1, 1] = P.YAxis.Y; M[1, 2] = P.ZAxis.Y; M[1, 3] = P.OriginY;
            M[2, 0] = P.XAxis.Z; M[2, 1] = P.YAxis.Z; M[2, 2] = P.ZAxis.Z; M[2, 3] = P.OriginZ;
            M[3, 0] = 0; M[3, 1] = 0; M[3, 2] = 0;  M[3, 3] = 1;

            return M;
        }

        public Transform Matrix2Transform(Matrix<double> M)
        {
            Transform T = new Transform();

            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                    T[i, j] = M[i, j];

            return T;
        }
    }

    public class Robot_Geometry:Robot_IK
    {
        enum t_state { init, new_branch, idx, new_path, collectX, collectY, collectZ, Xroad };

        public List<Mesh> JointGeometry;

  //      public List<Vector3d> RotationAxes;

  //      public List<Point3d> JointOrigin;

        public List<Plane> JointPlane;

        public Plane BasePlane;

        public Plane WorldCS;

        public Transform Xform2WorldCS;

        public Transform move2WorldCS;

        public Transform XformFromWorldCS;

        public Robot_Geometry()
        {
            JointGeometry = new List<Mesh>();
    //        RotationAxes = new List<Vector3d>();
    //        JointOrigin = new List<Point3d>();
            WorldCS = new Plane(new Point3d(0,0,0), new Vector3d(0,0,1));
            Xform2WorldCS = new Transform();
            move2WorldCS = new Transform();
            JointPlane = new List<Plane>();
        }

        public DataTree<Point3d> ParseMeshData(string data)
        {
            Rhino.UnitSystem currentUnit = Rhino.RhinoDoc.ActiveDoc.ModelUnitSystem;
            Rhino.UnitSystem robotUnit = Rhino.UnitSystem.Feet;

            double unitScale = Rhino.RhinoMath.UnitScale(robotUnit, currentUnit);

            StringBuilder path = new StringBuilder();
            StringBuilder point = new StringBuilder();

            Point3d point3d = new Point3d();
            DataTree<Point3d> DTree = new DataTree<Point3d>();
            
            t_state state = t_state.init;

            int i;

            for (i = 0; i < data.Length; i++)
            {
                switch (state)
                {
                    case t_state.init:
                        if (data[i] == '{')
                        {
                            path.Clear();
                            state = t_state.new_branch;
                        }
                        break;
                    case t_state.new_branch:
                        if (data[i] == '}')
                        {
                            DTree.EnsurePath(Convert.ToInt32(path.ToString()));
                            path.Clear();

                            state = t_state.idx;
                        }
                        else
                        {
                            path.Append(data[i]);
                        }
                        break;
                    case t_state.idx:
                        if (data[i] == '{')
                        {
                            // new item
                            state = t_state.collectX;
                        }
                        else
                        {
                            //eat up chars until new item starts
                        }
                        break;
                    case t_state.collectX:
                        if (data[i] == ',')
                        {
                            point3d.X = Convert.ToDouble(point.ToString()) * unitScale;
                            point.Clear();
                            state = t_state.collectY;
                        }
                        else
                        {
                            point.Append(data[i]);
                        }
                        break;
                    case t_state.collectY:
                        if (data[i] == ',')
                        {
                            point3d.Y = Convert.ToDouble(point.ToString()) * unitScale;
                            point.Clear();
                            state = t_state.collectZ;
                        }
                        else
                        {
                            point.Append(data[i]);
                        }
                        break;
                    case t_state.collectZ:
                        if (data[i] == '}')
                        {
                            point3d.Z = Convert.ToDouble(point.ToString()) * unitScale;
                            point.Clear();
                            DTree.Add(point3d);

                            state = t_state.Xroad;
                        }
                        else
                        {
                            point.Append(data[i]);
                        }
                        break;
                    case t_state.Xroad:
                        if (data[i] == '{')
                        {
                            // new branch
                            path.Clear();
                            state = t_state.new_branch;
                        }
                        else
                        {
                            // another point
                            if ('0' <= data[i] && data[i] <= '9')
                            {
                                state = t_state.idx;
                            }
                            else
                            {
                                //eat up chars
                            }
                        }
                        break;
                }
            }

            return DTree;
        }

        public Mesh MakeMesh(DataTree<Point3d> DTree)
        {
            // variables - mesh polygon, mesh face, the final mesh
            Polyline polyn;
            Mesh tempFace = new Mesh();
            Mesh entireMesh = new Mesh();

            //_______1____ branch cycle (for function)
            for (int j = 0; j < DTree.BranchCount; j++)
            {
                // renewing variables for a new branch
                tempFace = new Mesh();
                polyn = new Polyline();
                GH_Path pth = new GH_Path(j);

                //_____2____ indicies cycle (for function)
                for (int i = 0; i < DTree.Branches[j].Count; i++)
                {
                    polyn.Add(DTree[pth, i]);
                }//____2____

                //add first point to close the polyline
                polyn.Add(DTree[pth, 0]);
                tempFace = Mesh.CreateFromClosedPolyline(polyn);

                entireMesh.Append(tempFace);

            }//______1____

            return entireMesh;
        }
    }

    public class Robot:Robot_Geometry
    {
        public Robot()
        { }

    }

    public class Tool
    {
        public Mesh ToolMesh;
        public Plane EoT;
        public Plane EOT2Base;

        public Tool()
        {
            ToolMesh = new Mesh();
            EoT = new Plane();
            EOT2Base = new Plane();
        }
    }
}

