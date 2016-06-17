using System;
using System.Numerics;
using System.IO;
using System.Text;
using System.Collections.Generic;
using System.Threading;
using System.Windows.Forms;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Parameters;

using Grasshopper.GUI;

using Rhino;
using Rhino.Geometry;
using Rhino.DocObjects;
using Rhino.Collections;

using Robot_Elements;

using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;

namespace IKSolver
{
    enum TermType {Fine, Coarse, Continuous};
    enum MotionState { Accl_Start, Accl_End, Decl_Start, Decl_End, Motion_End, Joint_Interpolation };
    
    struct MotionProfile
    {
        public MotionState state;
        public Vector3d accl_step;
        public Vector3d decl_step;
        public long[] T;
        public long tick;
        public long motion_interval;
        public double rot_step;
        public double spin_step;
        public Point3d decel_start;
        public Plane Tgt_Pose;
        public Vector3d CommonNormal;
    };

    public class IKSolverComponent : GH_Component
    {        
        // 0.1 mm - for use in determining out of reach condition
        const double POSITION_ERROR = 0.1;
        
        // Menu items:
        double accel_time1 = 0.1;
        double accel_time2 = 0.1;
        double speed = 150.0;
        int vardecel = 1;
        TermType termination_type = TermType.Coarse;

        // State variables:
        Vector3d velocity = new Vector3d();
        Vector3d velocity_projected = new Vector3d();
        Vector3d accl = new Vector3d();
        int pos_idx;
        
        private MotionProfile AcclProfile;

        double intrpol_prd = 0.02;

        StreamWriter sw;
        double robot2CurrentScale = 1.0;
        double mm2CurrentScale = 1.0;
        const double ms2Seconds = 0.001;

        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public IKSolverComponent()
            : base("IK Solver", "IK",
                "Finds the joint angles of a 6 DoF robot for a given tool pose.",
                "TTU-DDF", "Robotics")
        {
            Rhino.UnitSystem currentUnit = Rhino.RhinoDoc.ActiveDoc.ModelUnitSystem;
            Rhino.UnitSystem robotUnit = Rhino.UnitSystem.Meters;
            Rhino.UnitSystem mmUnit = Rhino.UnitSystem.Millimeters;

            robot2CurrentScale = Rhino.RhinoMath.UnitScale(robotUnit, currentUnit);
            mm2CurrentScale = Rhino.RhinoMath.UnitScale(mmUnit, currentUnit);

            //AcclProfile = new MotionProfile();
            AcclProfile.state = MotionState.Motion_End;
            AcclProfile.T = new long[7];
            AcclProfile.T.Initialize();

            AcclProfile.tick = 0;
            AcclProfile.motion_interval = 0;

            velocity = Vector3d.Zero;
            velocity_projected = Vector3d.Zero;
            accl = Vector3d.Zero;
            
            //sw = File.CreateText("C:\\Users\\Swaps\\Desktop\\Motion.csv");

        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
              Param_ScriptVariable plane = new Param_ScriptVariable();
              Param_ScriptVariable robot = new Param_ScriptVariable();
              Param_ScriptVariable tool = new Param_ScriptVariable();

              pManager.AddParameter(plane, "EoT Plane", "P", "List of planes representing end of tool positionds and orientations", GH_ParamAccess.list);
              pManager.AddParameter(robot, "Robot", "R", "Robot definition", GH_ParamAccess.item);
              pManager.AddParameter(tool, "Tool", "T", "Tool definition", GH_ParamAccess.item);
              pManager.AddBrepParameter("Obstacles", "O", "A list of Breps of obstacles.", GH_ParamAccess.list);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            Param_ScriptVariable joint_angles = new Param_ScriptVariable();
            Param_ScriptVariable Jangles = new Param_ScriptVariable();

            pManager.AddParameter(joint_angles, "Joint Angles", "A", "Joint angles J1 through J6 in radians", GH_ParamAccess.tree);

            pManager.AddTextParameter("Errors", "E", "Errors", GH_ParamAccess.item);

            pManager.AddPlaneParameter("Plane", "P", "Plane representing the pose of the tool.", GH_ParamAccess.list);

            pManager.AddParameter(Jangles, "Joint Angles", "J", "Joint Angles at toolpath waypoints.", GH_ParamAccess.tree);

            pManager.AddBooleanParameter("IO output", "IO", "List of boolenas representing IO control output", GH_ParamAccess.list);

        }

        /****************************************************************************************************************************/

        /****************************************************************************************************************************/
        /* Custom Menu Items : Acceleration 1, Acceleration 2 and Speed                                                             */
        /****************************************************************************************************************************/
        protected override void AppendAdditionalComponentMenuItems(System.Windows.Forms.ToolStripDropDown menu)
        {
            ToolStripDropDown accl1_dropdown = new ToolStripDropDown();
            ToolStripDropDown accl2_dropdown = new ToolStripDropDown();
            ToolStripDropDown speed_dropdown = new ToolStripDropDown();

            ToolStripDropDownItem accl_1_item = new ToolStripMenuItem("Acceleration Time 1 (ms)");
            ToolStripDropDownItem accl_2_item = new ToolStripMenuItem("Acceleration Time 2 (ms)");
            ToolStripDropDownItem speed_item = new ToolStripMenuItem("Speed (mm/s)");

            GH_MenuTextBox accel_time1_TextBox = new GH_MenuTextBox(accl1_dropdown, accel_time1.ToString(), true);
            GH_MenuTextBox accel_time2_TextBox = new GH_MenuTextBox(accl2_dropdown, accel_time2.ToString(), true);
            GH_MenuTextBox speed_TextBox = new GH_MenuTextBox(speed_dropdown, speed.ToString(), true);

            accel_time1_TextBox.KeyDown += accel_time1_TextBox_KeyDown;
            accel_time1_TextBox.TextChanged += accel_time_TextBox_TextChanged;

            accel_time2_TextBox.KeyDown += accel_time2_TextBox_KeyDown;
            accel_time2_TextBox.TextChanged += accel_time_TextBox_TextChanged;

            speed_TextBox.KeyDown += speed_TextBox_KeyDown;
            speed_TextBox.TextChanged += accel_time_TextBox_TextChanged;

            accl_1_item.DropDown = accl1_dropdown;
            accl_2_item.DropDown = accl2_dropdown;
            speed_item.DropDown = speed_dropdown;

            menu.Items.Add(accl_1_item);
            menu.Items.Add(accl_2_item);
            menu.Items.Add(speed_item);

        }

        /****************************************************************************************************************************/
        /* Updates speed
        /****************************************************************************************************************************/
        void speed_TextBox_KeyDown(GH_MenuTextBox sender, KeyEventArgs e)
        {
            double val;

            // If 'Commit Changes', update the speed
            if (e.KeyValue == 13)
            {
                if (double.TryParse(sender.Text, out val))
                {
                    speed = val * mm2CurrentScale;
                }
            }
        }

        /****************************************************************************************************************************/
        /* Updates acceleration time 2                                                                                              */
        /****************************************************************************************************************************/
        void accel_time2_TextBox_KeyDown(GH_MenuTextBox sender, KeyEventArgs e)
        {
            double val;

            // If 'Commit Changes', update the acceleration time period
            if (e.KeyValue == 13)
            {
                if (double.TryParse(sender.Text, out val))
                {
                    accel_time2 = val * ms2Seconds;
                }
            }
        }

        /****************************************************************************************************************************/
        /* Updates acceleration time 1                                                                                              */
        /****************************************************************************************************************************/
        void accel_time1_TextBox_KeyDown(GH_MenuTextBox sender, System.Windows.Forms.KeyEventArgs e)
        {
            double val;

            // If 'Commit Changes', update the acceleration time period
            if (e.KeyValue == 13)
            {
                if (double.TryParse(sender.Text, out val))
                {
                    accel_time1 = val * ms2Seconds;
                }
            }

        }
        /****************************************************************************************************************************/
        /* A common handler to handle text changed event of all text boxes                                                          */
        /****************************************************************************************************************************/
        private void accel_time_TextBox_TextChanged(GH_MenuTextBox sender, string text)
        {
            double val;
            string saved = text;

            // Check for numeric text and remove non-numeric input
            if (!double.TryParse(text, out val) && text != ".")
            {
                sender.Text = "";

                if (saved.Length > 1)
                {
                    SendKeys.SendWait(saved.Remove(saved.Length - 1));
                }
            }
        }
        /****************************************************************************************************************************/

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double[] x = new double[6];
            double[] z = new double[6];

            string error = "";

            Robot robot = new Robot();
            Tool tool = new Tool();

            Matrix<double> pose = Matrix<double>.Build.Dense(12, 1, 0);
            Matrix<double> pose_d = Matrix<double>.Build.Dense(6, 1, 0);
            Matrix<double> pose_final = Matrix<double>.Build.Dense(12, 1, 0);
            Matrix<double> Je_inv = Matrix<double>.Build.Dense(6, 6, 0);
            Matrix<double> JointAngle_delta = Matrix<double>.Build.Dense(6, 1, 0);
    
            List<double> Robot_para = new List<double>();
            List<Plane> planes = new List<Plane>();
            List<List<double>> angle_List = new List<List<double>>();
            List<List<double>> Jangle_List = new List<List<double>>();

            List<bool> IO_lst = new List<bool>();

            List<Brep> ObstacleLst = new List<Brep>();

            Matrix<double> I6 = Matrix<double>.Build.Dense(6, 6);
            I6[0, 0] = 1; I6[1, 1] = 1; I6[2, 2] = 1;
            I6[3, 3] = 1; I6[4, 4] = 1; I6[5, 5] = 1;

            Vector3d CollisionDir = new Vector3d();

            Matrix<double> I3 = Matrix<double>.Build.Dense(3, 3);
            I3[0, 0] = 1000; I3[1, 1] = 1000; I3[2, 2] = 1000;

            int err_code = 0;

            AcclProfile.tick = 0;
            AcclProfile.motion_interval = 0;

            AcclProfile.state = MotionState.Joint_Interpolation;

            velocity = Vector3d.Zero;
            accl = Vector3d.Zero;

            //pos_idx = 0;

            for( int i = 0; i < 6; i++){
                angle_List.Add(new List<double>());
                Jangle_List.Add(new List<double>());
            }

            Rhino.UnitSystem currentUnit = Rhino.RhinoDoc.ActiveDoc.ModelUnitSystem;
            Rhino.UnitSystem robotUnit = Rhino.UnitSystem.Meters;

            double unitScale = Rhino.RhinoMath.UnitScale(robotUnit, currentUnit);

            Matrix<double> TCP = Matrix<double>.Build.Dense(4, 4, 0);

            List<Plane> ToolPln = new List<Plane>();

            // Read the inputs:
            DA.GetDataList<Plane>(0, planes);
            DA.GetData<Robot>(1, ref robot);
            DA.GetData<Tool>(2, ref tool);
            DA.GetDataList<Brep>(3, ObstacleLst);

            Brep Obs = new Brep();
            Obs = ObstacleLst[0];

            Obs.Transform(robot.Xform2WorldCS);

            Point3d Obs_cntr = Rhino.Geometry.VolumeMassProperties.Compute(Obs).Centroid;

            Plane PT_hotwire = new Plane(tool.EOT2Base);//Point3d.Origin, Vector3d.ZAxis);

            Matrix<double> JointAngles =  Matrix<double>.Build.Dense(6,1,0);
            double J2Angle = robot.JointAngles[1, 0];

            Matrix<double> J = Matrix<double>.Build.Dense(6, 6);
            Matrix<double> Jc = Matrix<double>.Build.Dense(3, 6);

            JointAngles = robot.JointAngles;
            
            // xForm from Robot to World CS
            Transform Back2World = robot.PlanetoTransform(robot.WorldCS);

            // Home position:
            for (int i = 0; i < 6; i++)
            {
                robot.JointAngles[i, 0] = 0;
            }

            robot.JointAngles[1, 0] = -Math.PI / 2;
            
            for (int i = 0; i < robot.JointAngles.RowCount && err_code == 0; i++)
            {
                if (i == 1)
                {
                    angle_List[i].Add(robot.JointAngles[i, 0] + Math.PI / 2);
                }
                else
                {
                    angle_List[i].Add(robot.JointAngles[i, 0]);
                }

            }

            IO_lst.Add(false);

           // robot.JointAngles[1, 0] -= Math.PI / 2;
            for (int pln_cnt = 0; pln_cnt < planes.Count; pln_cnt++)
            {
                int j_cnt = 0;
                List<double> Q = new List<double>();

                do
                {
                    robot.Update_Pose();
                    robot.Update_Je();
                    
                    Plane tool_base = new Plane(robot.Position, robot.Orientation.XAxis, robot.Orientation.YAxis);
                    tool_base.Transform(Back2World);
                    ToolPln.Add(tool_base);


                    switch (AcclProfile.state)
                    {
                        case MotionState.Joint_Interpolation:
                        case MotionState.Motion_End:

                            Plane TCP1 = new Plane(planes[pln_cnt]);
                            
                            Vector3d Xaxis_XY = new Vector3d(Vector3d.CrossProduct(TCP1.YAxis, -Vector3d.XAxis));

                            Xaxis_XY.Unitize();

                            Plane TCP2 = new Plane(TCP1.Origin, Xaxis_XY, TCP1.YAxis);
                            
                            ///Matrix<double> Tgt = Matrix<double>.Build.Dense(4, 4, 0);

                            RPlane RTgt = new RPlane(TCP1);
                            Transform xFormrTgt = RTgt.PlanetoTransform();

                            //Tgt = robot.Plane2Matrix(TCP2);

                            //Plane TargetPln = new Plane(TCP1);

                            Plane TargetPln = new Plane(PT_hotwire);

                            TargetPln.Transform(xFormrTgt);
                            
                            TargetPln.Transform(robot.Xform2WorldCS);
                            
                            Plane pos_current = new Plane(robot.Position, robot.Orientation.XAxis, robot.Orientation.YAxis);

                            double yy = Vector3d.Multiply(robot.Orientation.XAxis, robot.Orientation.YAxis);
                            double zz = Vector3d.Multiply(robot.Orientation.YAxis, robot.Orientation.ZAxis);

                            if (MotionState.Joint_Interpolation == AcclProfile.state)
                            {
                                if (j_cnt < 10) //&& pln_cnt == 0)
                                {
                                    //AcclProfile.state = MotionState.Joint_Interpolation;

                                    //TargetPln.Rotate(Math.PI, TargetPln.XAxis);                                    

                                    if (j_cnt == 0)
                                    {
                                        Joint_Interpolation(TargetPln, Q);

                                        for (int i = 0; i < 6; i++)
                                        {
                                            Q[i] = (Q[i] - robot.JointAngles[i, 0]) / (10);
                                        }
                                    }
                                    

                                    for (int i = 0; i < 6; i++)
                                    {
                                        robot.JointAngles[i, 0] += Q[i];
                                    }

                                    j_cnt++;
                                }

                                if(j_cnt >= 10)
                                {
                                    AcclProfile.state = MotionState.Motion_End;
                                    break;
                                    //err_code = Motion_Profile(pos_current, TargetPln, velocity);
                                }
                            }

                            //AcclProfile.tick = 0;

                            if(AcclProfile.state == MotionState.Motion_End)
                            {
                                //err_code = Motion_Profile(pos_current, TargetPln, velocity);
                                Motion_Profile(pos_current, TargetPln, velocity);
                                AcclProfile.state = MotionState.Accl_Start;

                                /*
                                if (err_code == 0)
                                {
                                    AcclProfile.state = MotionState.Accl_Start;
                                    //AcclProfile.state = MotionState.Decl_End;
                                    AcclProfile.tick = 0;
                                }
                                */
                            }

                            break;
                    }

                    if (MotionState.Motion_End != AcclProfile.state && err_code == 0 && AcclProfile.state != MotionState.Joint_Interpolation)
                    {
                        if (MotionState.Decl_End != AcclProfile.state)
                        {
                            double delta_d;// = (velocity * intrpol_prd + 0.5 * accl * intrpol_prd * intrpol_prd).Length;

                            double remaining_d = (AcclProfile.Tgt_Pose.Origin - robot.Position).Length;

                            if (Math.Abs(remaining_d) < .10)
                            {
                                AcclProfile.state = MotionState.Motion_End;
                            }
                                delta_d = remaining_d / 20;

                                velocity.X = (AcclProfile.Tgt_Pose.OriginX - robot.Position.X) / 20 / intrpol_prd;
                                velocity.Y = (AcclProfile.Tgt_Pose.OriginY - robot.Position.Y) / 20 / intrpol_prd;
                                velocity.Z = (AcclProfile.Tgt_Pose.OriginZ - robot.Position.Z) / 20 / intrpol_prd;
       
                            double delta_Zaxis;
                            Vector3d v1 = new Vector3d(robot.Orientation.ZAxis);
                            Vector3d v2 = new Vector3d(AcclProfile.Tgt_Pose.ZAxis);

                            v1.Unitize();
                            v2.Unitize();
                            
                            Vector3d commonnormal = Vector3d.CrossProduct(v1,v2);//robot.Orientation.ZAxis, AcclProfile.Tgt_Pose.ZAxis);

                            commonnormal.Unitize();

                            delta_Zaxis = Math.Acos(Vector3d.Multiply(v1,v2));//robot.Orientation.ZAxis, AcclProfile.Tgt_Pose.ZAxis));

                            Plane final_ort = new Plane(robot.Position, robot.Orientation.XAxis, robot.Orientation.YAxis);
                            final_ort.Rotate(delta_Zaxis, commonnormal);

                            double delta_Xaxis;
                            Vector3d v3 = new Vector3d(final_ort.XAxis);
                            Vector3d v4 = new Vector3d(AcclProfile.Tgt_Pose.XAxis);

                            v3.Unitize();
                            v4.Unitize();

                            Vector3d commonnormal2 = Vector3d.CrossProduct(v3, v4);//robot.Orientation.ZAxis, AcclProfile.Tgt_Pose.ZAxis);

                            commonnormal2.Unitize();

                            delta_Xaxis = Math.Acos(Vector3d.Multiply(v3,v4));//final_ort.XAxis, AcclProfile.Tgt_Pose.XAxis));

                            double rot_velocity = (delta_Zaxis * delta_d) / remaining_d;// / intrpol_prd;
                            double spin_velocity = (delta_Xaxis * delta_d) / remaining_d;// / intrpol_prd;

                            Plane pose_delta = new Plane((Point3d)velocity, Vector3d.Zero, Vector3d.Zero);
                            Plane pose_zero = new Plane(Point3d.Origin, robot.Orientation.XAxis, robot.Orientation.YAxis);
                            Plane pose_new = new Plane(Point3d.Origin, robot.Orientation.XAxis, robot.Orientation.YAxis);


                            pose_new.Rotate(rot_velocity, commonnormal);// AcclProfile.CommonNormal);
                            pose_new.Rotate(spin_velocity, commonnormal2);// pose_new.ZAxis);

                            RPlane Rpose_new = new RPlane(pose_new);
                            RPlane Rpose_zero = new RPlane(pose_zero);
                            RPlane Rpose_delta = new RPlane(pose_delta);
                            RPlane Rpose = new RPlane(new Plane(robot.Position, robot.Orientation.XAxis, robot.Orientation.YAxis));

                            Matrix <double>dR = Rpose_delta.Plane2Matrix() + Rpose_new.Plane2Matrix() - Rpose_zero.Plane2Matrix();
                            Matrix <double>R = Rpose.Plane2Matrix();

                            pose_d[0, 0] = dR[0, 3];
                            pose_d[1, 0] = dR[1, 3];
                            pose_d[2, 0] = dR[2, 3];

                            dR = dR / intrpol_prd;

                            Matrix<double> Sw = Matrix<double>.Build.Dense(3, 3);

                            Sw = dR.SubMatrix(0, 3, 0, 3) * R.SubMatrix(0, 3, 0, 3).Transpose();


                            pose_d[3, 0] = Sw[2, 1];
                            pose_d[4, 0] = Sw[0, 2];
                            pose_d[5, 0] = Sw[1, 0];

                            Vector3d O6 = new Vector3d(robot.J6_pose[0, 3], robot.J6_pose[1, 3], robot.J6_pose[2, 3]);
                            Vector3d Z0 = new Vector3d(Vector3d.ZAxis); //(robot.BasePlane.ZAxis);
                            //Vector3d Z0 = new Vector3d(robot.J1_pose[0, 2], robot.J1_pose[1, 2], robot.J1_pose[2, 2]);

                            Vector3d vel = Vector3d.CrossProduct(Z0, O6);

                            J[0, 0] = vel.X;    J[1, 0] = vel.Y;    J[2, 0] = vel.Z;
                            J[3, 0] = Z0.X;     J[4, 0] = Z0.Y;     J[5, 0] = Z0.Z;

                            Vector3d O1 = new Vector3d(robot.J1_pose[0, 3], robot.J1_pose[1, 3], robot.J1_pose[2, 3]);
                            Vector3d Z1 = new Vector3d(robot.J1_pose[0, 2], robot.J1_pose[1, 2], robot.J1_pose[2, 2]);

                            vel = Vector3d.CrossProduct(Z1, O6-O1);

                            J[0, 1] = vel.X;    J[1, 1] = vel.Y;    J[2, 1] = vel.Z;
                            J[3, 1] = Z1.X;     J[4, 1] = Z1.Y;     J[5, 1] = Z1.Z;

                            Vector3d O2 = new Vector3d(robot.J2_pose[0, 3], robot.J2_pose[1, 3], robot.J2_pose[2, 3]);
                            Vector3d Z2 = new Vector3d(robot.J2_pose[0, 2], robot.J2_pose[1, 2], robot.J2_pose[2, 2]);

                            vel = Vector3d.CrossProduct(Z2, O6 - O2);

                            J[0, 2] = vel.X;    J[1, 2] = vel.Y;    J[2, 2] = vel.Z;
                            J[3, 2] = Z2.X;     J[4, 2] = Z2.Y;     J[5, 2] = Z2.Z;

                            Vector3d O3 = new Vector3d(robot.J3_pose[0, 3], robot.J3_pose[1, 3], robot.J3_pose[2, 3]);
                            Vector3d Z3 = new Vector3d(robot.J3_pose[0, 2], robot.J3_pose[1, 2], robot.J3_pose[2, 2]);

                            vel = Vector3d.CrossProduct(Z3, O6 - O3);

                            J[0, 3] = vel.X;    J[1, 3] = vel.Y;    J[2, 3] = vel.Z;
                            J[3, 3] = Z3.X;     J[4, 3] = Z3.Y;     J[5, 3] = Z3.Z;

                            Vector3d O4 = new Vector3d(robot.J4_pose[0, 3], robot.J4_pose[1, 3], robot.J4_pose[2, 3]);
                            Vector3d Z4 = new Vector3d(robot.J4_pose[0, 2], robot.J4_pose[1, 2], robot.J4_pose[2, 2]);

                            vel = Vector3d.CrossProduct(Z4, O6 - O4);

                            J[0, 4] = vel.X;    J[1, 4] = vel.Y;    J[2, 4] = vel.Z;
                            J[3, 4] = Z4.X;     J[4, 4] = Z4.Y;     J[5, 4] = Z4.Z;

                            Vector3d O5 = new Vector3d(robot.J5_pose[0, 3], robot.J5_pose[1, 3], robot.J5_pose[2, 3]);
                            Vector3d Z5 = new Vector3d(robot.J5_pose[0, 2], robot.J5_pose[1, 2], robot.J5_pose[2, 2]);

                            vel = Vector3d.CrossProduct(Z5, O6 - O5);

                            J[0, 5] = vel.X;    J[1, 5] = vel.Y;    J[2, 5] = vel.Z;
                            J[3, 5] = Z5.X;     J[4, 5] = Z5.Y;     J[5, 5] = Z5.Z;

                            Point3d P1 = new Point3d(robot.J3_pose[0,3], robot.J3_pose[1,3], robot.J3_pose[2,3]);
                            Point3d P2 = new Point3d(robot.J4_pose[0,3], robot.J4_pose[1,3], robot.J4_pose[2,3]);

                            Point3d Pc = new Point3d();
                            double Dc = new double();

                            Critical_point(ObstacleLst[0], Obs_cntr, P1, P2, ref Pc, ref Dc);
/*
                            Point3d pt = Pc;
 * 
                            pt.Transform(robot.XformFromWorldCS);

                            string cmd = "Line " + pt.ToString() + " ";

                            pt = Obs_cntr;
                            pt.Transform(robot.XformFromWorldCS);

                            cmd += pt.ToString() + " ";

                            Rhino.RhinoApp.RunScript(cmd, true);
*/
                            if (false)//Dc < 100)
                            {
                                Vector3d Oc = new Vector3d(Pc.X, Pc.Y, Pc.Z);

                                CollisionDir = Obs_cntr - Pc;
                                CollisionDir.Unitize();

                                vel = Vector3d.CrossProduct(Z0, Oc);

                                Jc[0, 0] = vel.X * CollisionDir.X; Jc[1, 0] = vel.Y * CollisionDir.Y; Jc[2, 0] = vel.Z * CollisionDir.Z;

                                vel = Vector3d.CrossProduct(Z1, Oc-O1);

                                Jc[0, 1] = vel.X * CollisionDir.X; Jc[1, 1] = vel.Y * CollisionDir.Y; Jc[2, 1] = vel.Z * CollisionDir.Z;


                                vel = Vector3d.CrossProduct(Z2, Oc-O2);

                                Jc[0, 2] = vel.X * CollisionDir.X; Jc[1, 2] = vel.Y * CollisionDir.Y; Jc[2, 2] = vel.Z * CollisionDir.Z;

                                /*
                                vel = Vector3d.CrossProduct(Z3, Oc);
                                
                                Jc[0, 3] = vel.X; Jc[1, 3] = vel.Y; Jc[2, 3] = vel.Z;

                                vel = Vector3d.CrossProduct(Z4, Oc);

                                Jc[0, 4] = vel.X; Jc[1, 4] = vel.Y; Jc[2, 4] = vel.Z;

                                vel = Vector3d.CrossProduct(Z5, Oc);

                                Jc[0, 5] = vel.X; Jc[1, 5] = vel.Y; Jc[2, 5] = vel.Z;
                                
                                 */

                                Je_inv = (J.Transpose() * I6 * J + Jc.Transpose() * I3 * Jc + 0.01 * I6).Inverse();

                                JointAngle_delta = Je_inv * J.Transpose() * I6 * pose_d;

                                robot.JointAngles += JointAngle_delta * intrpol_prd;
                            }
                            else
                            {
                                Je_inv = (J.Transpose() * I6 * J + 0.01 * I6).Inverse();

                                JointAngle_delta = Je_inv * J.Transpose() * I6 * pose_d;

                                robot.JointAngles += JointAngle_delta * intrpol_prd;
                            }
                            
                        }
                                        
 /*                       if (J.Rank() == 6)
                        {
                            //Je_inv = J.Inverse();

                            //JointAngle_delta = Je_inv * pose_d;

                            Je_inv = (J.Transpose() * I6 * J + Jc.Transpose() * I3 * Jc + 0.01 * I6).Inverse();

                            JointAngle_delta = Je_inv * J.Transpose() * I6 * pose_d;
                            
                            robot.JointAngles += JointAngle_delta * intrpol_prd;
                        }
                        else
                        {
                            err_code = 1;
                        }
   */                     
                    }

                    // Check for out of reach condition
                    /*           if (!(pose[9,0].AlmostEqual(pose_final[9,0], POSITION_ERROR) &&
                                pose[10, 0].AlmostEqual(pose_final[10, 0], POSITION_ERROR) &&
                                pose[11, 0].AlmostEqual(pose_final[11, 0], POSITION_ERROR)))
                            {
                                error = error + "Out of Reach.\n";

                                // Restore previous valid configuration
                                robot.JointAngles = JointAngles;
                                robot.JointAngles[1, 0] = J2Angle;
                            }
                            else
                    */
                    if(err_code == 0)
                    {
                        //robot.JointAngles[1, 0] += Math.PI / 2;
                        robot.JointAngles = robot.JointAngles % (2 * Math.PI);

                        JointAngles = robot.JointAngles;
                    }

                    //angles.Clear();

                    for (int i = 0; i < robot.JointAngles.RowCount && err_code == 0; i++)
                    {
                        if (i == 1)
                        {
                            angle_List[i].Add(robot.JointAngles[i, 0] + Math.PI / 2);
                        }
                        else
                        {
                            angle_List[i].Add(robot.JointAngles[i, 0]);
                        }

                    }

                    if (pln_cnt > 1)
                    {
                        IO_lst.Add(true);
                    }
                    else
                    {
                        IO_lst.Add(false);
                    }

                } while (AcclProfile.state != MotionState.Motion_End && err_code == 0);

                for (int i = 0; i < robot.JointAngles.RowCount && err_code == 0; i++)
                {
                    if (i == 1)
                    {
                        Jangle_List[i].Add(robot.JointAngles[i, 0] + Math.PI / 2);
                    }
                    else
                    {
                        Jangle_List[i].Add(robot.JointAngles[i, 0]);
                    }

                }
            }

            DataTree<double> angle_tree = new DataTree<double>();
            DataTree<double> Jangle_tree = new DataTree<double>();

            for (int i = 0; i < angle_List.Count; i++)
            {
                angle_tree.AddRange(angle_List[i], new GH_Path(i));                
            }

            for (int i = 0; i < Jangle_List.Count; i++)
            {
                Jangle_tree.AddRange(Jangle_List[i], new GH_Path(i));
            }

            if (-1 == err_code)
            {
                error = "Distance too short";
            }

            DA.SetDataTree(0, angle_tree);
            DA.SetData(1, error);
            DA.SetDataList(2, ToolPln);
            DA.SetDataTree(3, Jangle_tree);
            DA.SetDataList(4, IO_lst);
        }

        private void Critical_point(Brep Obs, Point3d Centroid, Point3d p1, Point3d p2, ref Point3d Pc, ref double Dc)
        {
            //Point3d Centroid = Rhino.Geometry.VolumeMassProperties.Compute(Obs).Centroid;
            Line link = new Line(p1, p2);
            Point3d Pnt_on_link = link.ClosestPoint(Centroid, true);
            Point3d Pnt_on_Obs = Obs.ClosestPoint(Pnt_on_link);

            Pc = link.ClosestPoint(Pnt_on_Obs, true);
            Dc = Pc.DistanceTo(Pnt_on_Obs);
                        
            return;
        }

        /****************************************************************************************************************************/
        /* Joint_Interpolation()
        /****************************************************************************************************************************/
        private void Joint_Interpolation(Plane tgt, List<double> Q)
        {
            Rhino.UnitSystem currentUnit = Rhino.RhinoDoc.ActiveDoc.ModelUnitSystem;
            Rhino.UnitSystem robotUnit = Rhino.UnitSystem.Meters;

            double unitScale = Rhino.RhinoMath.UnitScale(robotUnit, currentUnit);
            double xx, zz, L1, L2, D, temp;
            double q1, q2, q3, q4 ,q5, q6, q2_i;

            q1 = q2 = q3 = q4= q5= q6= 0.0;

            double x = tgt.OriginX;
            double y = tgt.OriginY;
            double z = tgt.OriginZ;

            RPlane Rtgt = new RPlane(tgt);
            Transform RxForm = Rtgt.PlanetoTransform();
            RxForm.M03 = 0;
            RxForm.M13 = 0;
            RxForm.M23 = 0;

            Point3d WO = new Point3d(0, 0, -0.175 * unitScale);

            WO.Transform(RxForm);

            x = x + WO.X;
            y = y + WO.Y;
            z = z + WO.Z;

            q1 = Math.Atan2(y,x);

            x = Math.Sqrt(x*x + y*y) - 0.150 * unitScale;

            xx = x * x;
            zz = z * z;

            L1 = 0.870 * unitScale;
            L2 = Math.Sqrt(1.016 * 1.016 + 0.170 * 0.170) * unitScale;

            D = (xx + zz - L1 * L1 - L2 * L2) / (-2 * L1 * L2);

            if (Math.Abs(D) <= 1.0)
            {
                temp = Math.Atan2(Math.Sqrt(1 - D * D), D);
                q3 = Math.PI - temp - Math.Atan2(1016, 170);

                D = (L2 * L2 - L1 * L1 - xx -zz) / (-2 * L1 * Math.Sqrt(xx+zz));
                q2 = Math.Atan2(z, Math.Sqrt(xx)) + Math.Atan2(Math.Sqrt(1-D*D), D);

                //q2 = -q2 + Math.PI / 2;
                //q2 = q2 - Math.PI / 2;

                q2 = -q2;
                q3 = -q3;

                /* Orientation */
                q4 = Math.Atan2((-Math.Sin(q1) * tgt.ZAxis.X + Math.Cos(q1) * tgt.ZAxis.Y),
                    (Math.Cos(q1) * Math.Cos(q2-q3) * tgt.ZAxis.X + Math.Sin(q1) * Math.Cos(q2-q3) * tgt.ZAxis.Y - Math.Sin(q2-q3) * tgt.ZAxis.Z));

                q6 = Math.Atan2((Math.Cos(q1) * Math.Sin(q2 - q3) * tgt.YAxis.X + Math.Sin(q1) * Math.Sin(q2 - q3) * tgt.YAxis.Y
                     + Math.Cos(q2 - q3) * tgt.YAxis.Z), Math.Cos(q1) * Math.Sin(q2 - q3) * tgt.XAxis.X + Math.Sin(q1) * Math.Sin(q2 - q3)
                     * tgt.XAxis.Y + Math.Cos(q2 - q3) * tgt.XAxis.Z);

                D = Math.Cos(q1) * Math.Sin(q2-q3) * tgt.ZAxis.X + Math.Sin(q1) * Math.Sin(q2-q3) * tgt.ZAxis.Y
                    + Math.Cos(q2-q3) * tgt.ZAxis.Z;

                q5 = Math.Atan2(Math.Sqrt(1-D * D), D);

                q5 = -q5 + Math.PI;

                //q2 = q2 + Math.PI / 2;

                
            }

            Q.Add(q1);
            Q.Add(q2);// + Math.PI/2);
            Q.Add(q3);
            Q.Add(q4);
            Q.Add(q5);
            Q.Add(q6);

        }
/*

            List<double> joint_times = new List<double>();
            int[] joint_speeds = new int[6];
            
            joint_speeds[0] = 175;
            joint_speeds[1] = 175;
            joint_speeds[2] = 175;
            joint_speeds[3] = 250;
            joint_speeds[4] = 250;
            joint_speeds[5] = 355;

            for(int i=0; i < robot.JointAngles.RowCount; i++)
            {
                joint_times.Add(robot.JointAngles[i, 0]/joint_speeds[i]);
            }


        }*/
            /****************************************************************************************************************************/
            /* Motion_Profile()
            /****************************************************************************************************************************/
            private int Motion_Profile(Plane pos1, Plane pos2, Vector3d V_cur)
            {
                Vector3d acc_peak = new Vector3d();
                Vector3d decl_peak = new Vector3d();
                Vector3d V_tgt = new Vector3d();
                Vector3d acc_step = new Vector3d(Vector3d.Zero);
                double distance;

                AcclProfile.Tgt_Pose = pos2;

                AcclProfile.T[0] = (long)(Math.Min(accel_time1, accel_time2) / intrpol_prd);
                AcclProfile.T[1] = (long)(Math.Max(accel_time1, accel_time2) / intrpol_prd);
                AcclProfile.T[2] = AcclProfile.T[0] + AcclProfile.T[1];

                Vector3d direction = (pos2.Origin - pos1.Origin);
                
                //distance = direction.Length;

                direction.Unitize();

                V_tgt = speed * direction;

                acc_peak = (V_tgt - V_cur) / Math.Max(accel_time1, accel_time2);

                if(Math.Min(accel_time1,accel_time2) > 0.0001){
                    AcclProfile.accl_step = acc_peak * intrpol_prd / Math.Min(accel_time1,accel_time2);
                }
                else
                {
                    // acc_step is peak value, since acceleration time is zero
                    AcclProfile.accl_step = acc_peak;

                    // Acceleration is reached in a single step
                    // Applies to deceleration profile too
                    AcclProfile.T[0] = 1; 
                }

                decl_peak = (Vector3d.Zero - V_tgt) / Math.Max(accel_time1, accel_time2);

                if (Math.Min(accel_time1, accel_time2) > 0.0001)
                {
                    AcclProfile.decl_step = decl_peak * intrpol_prd / Math.Min(accel_time1, accel_time2);
                }
                else
                {
                    // decl_step is peak value, since deceleration time is zero
                    AcclProfile.decl_step = decl_peak;
                }

                Vector3d V = V_tgt;
                Vector3d A = Vector3d.Zero;
                Vector3d D = Vector3d.Zero;

                for (long i = 0 ; i < AcclProfile.T[2]; i++)
                {
                    if (i < AcclProfile.T[0])
                    {
                        A += AcclProfile.decl_step;
                    }
                    else if (i >= AcclProfile.T[0] && i < AcclProfile.T[1])
                    {
                        // Do nothing in the straight part of acceleration profile //
                    }
                    else if (i >= AcclProfile.T[1])
                    {
                        A -= AcclProfile.decl_step;
                    }

                    V += A * intrpol_prd;
                    D += V * intrpol_prd;
                }

                if (D.Length * 2 <= (pos2.Origin - pos1.Origin).Length)
                {
                    AcclProfile.decel_start = pos2.Origin - D;
                }
                else
                {
                    return -1;
                }
                    
                AcclProfile.CommonNormal = Vector3d.CrossProduct(pos1.ZAxis, pos2.ZAxis);
                AcclProfile.CommonNormal.Unitize();
                //AcclProfile.CommonNormal.Reverse();

                return 0;
            }


        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return Properties.Resources.IKSolverIcon;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("{46265b0e-c753-4c3e-8a79-a7970a3d380b}"); }
        }
    }
}
