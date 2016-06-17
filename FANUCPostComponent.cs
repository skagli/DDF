using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using System.Windows.Forms;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;

using Grasshopper.GUI;

using Robot_Math;

using Rhino;
using Rhino.Geometry;
//using Rhino.DocObjects;
//using Rhino.Collections;

using Robot_Elements;

namespace FANUCPost
{
    public class FANUCPostComponent : GH_Component
    {  
        String path = "Enter path and file name";

        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public FANUCPostComponent()
            : base("FANUCPost", "FPost",
                "Post processor for FANUC robot",
                "TTU-DDF", "Robotics")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            Param_ScriptVariable P = new Param_ScriptVariable();
            Param_ScriptVariable R = new Param_ScriptVariable();

            pManager.AddParameter(R, "Robot", "R", "Robot parameters obtained from the robot component.", GH_ParamAccess.item);
            pManager.AddParameter(P, "Planes", "TP", "List of planes that define the complete toolpath.", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Planes", "P", "List of planes that define the toolpath minus approach/repose poses.", GH_ParamAccess.tree);
            pManager.AddAngleParameter("Angles", "J", "A six-branch tree with joint angles for each pose on the toolpath.", GH_ParamAccess.tree);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {

        }

        /****************************************************************************************************************************/
        /* Custom Menu Items : Path                                                             */
        /****************************************************************************************************************************/
        public override void AppendAdditionalMenuItems(System.Windows.Forms.ToolStripDropDown menu)
        {
            ToolStripDropDown path_dropdown = new ToolStripDropDown();

            ToolStripDropDownItem path_item = new ToolStripMenuItem("Path and name of .LS file");

            GH_MenuTextBox path_textbox = new GH_MenuTextBox(path_dropdown, path, true);

            //path_textbox.KeyDown += path_textbox_keydown;
            path_textbox.TextChanged += path_textbox_txtchg;

            path_item.DropDown = path_dropdown;

            menu.Items.Add(path_item);
        }

        void path_textbox_txtchg(GH_MenuTextBox sender, string text)
        {
            int pos;

            // Ensure the full path with file name ending in '.LS' is provided//
            if ((pos = string. Compare(text, text.Length-3,".LS",0,3, true)) == 0) 
            {
                try
                {

                    GH_Document gh_doc = OnPingDocument();

                    path = Path.GetFullPath(text);

                    gh_doc.ExpireSolution();
                    gh_doc.NewSolution(false);
                }

                catch
                {
                    //sender.Text = "Enter the path";
                }
            }
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        enum state { App_Rep, Tp };
        enum intr_type { Joint, Linear };
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int i = 0;
            int LineCount = 1;
            int PositionCount = 1;
            state state = state.App_Rep;
            intr_type motion_type = intr_type.Joint;
            string motion_str = "J";
            string term_type = "FINE";
            string speed = "15";
            string tp_speed = "15";
            string app_retract_spd = "50";
            string speed_unit = "%";
            int error = 0;
            bool switch_spd = false;

            Robot robot = new Robot();

            List<Plane> lst = new List<Plane>();
            GH_Structure<GH_Plane> TreePln = new GH_Structure<GH_Plane>();
            GH_Structure<GH_Number> TreeAng = new GH_Structure<GH_Number>();

            Plane plane = Plane.Unset;

            List<Plane> lst_out = new List<Plane>();

            Robot_Orientation orient = new Robot_Orientation();

            Rhino.UnitSystem currentUnit = Rhino.RhinoDoc.ActiveDoc.ModelUnitSystem;
            Rhino.UnitSystem robotUnit = Rhino.UnitSystem.Millimeters;

            double unitScale = Rhino.RhinoMath.UnitScale(currentUnit, robotUnit);                
            

            System.IO.StreamWriter file = new System.IO.StreamWriter(path, false);
            StringBuilder sb = new StringBuilder();
                        
            DA.GetData<Robot>(0, ref robot);
            DA.GetDataList<Plane>(1, lst);
            DA.GetDataTree<GH_Plane>(2, out TreePln);
            DA.GetDataTree<GH_Number>(3, out TreeAng);

            file.WriteLine("/PROG  POST");
            file.WriteLine("/ATTR");
            file.WriteLine("TCD:  STACK_SIZE        = 0,");
            file.WriteLine("      TASK_PRIORITY     = 50,");
            file.WriteLine("      TIME_SLICE        = 0,");
            file.WriteLine("      BUSY_LAMP_OFF     = 0,");
            file.WriteLine("      ABORT_REQUEST     = 0,");
            file.WriteLine("      PAUSE_REQUEST     = 0;");
            file.WriteLine("DEFAULT_GROUP    = 1,*,*,*,*;");
            file.WriteLine("CONTROL_CODE    = 00000000 00000000;");

            file.WriteLine("\n/MN");

            file.WriteLine("{0,4:D}:  UFRAME_NUM = 1 ;", LineCount++);
            file.WriteLine("{0,4:D}:  UTOOL_NUM = 1 ;", LineCount++);

            file.WriteLine("{0,4:D}:{1} P[{2:D}] {3:D}{4} {5} ;", LineCount++, "J", PositionCount++, "50", "%", "FINE");
            //file.WriteLine("{0,4:D}:{1} P[{2:D}] {3:D}{4} {5} ;", LineCount++, "J", PositionCount++, "50", "%", "FINE");


            if (motion_type == intr_type.Joint)
            {
                motion_str = "J";
            }
            else
            {
                motion_str = "L";
            }

            int pln_idx = 0;
            int brnch = 0;
            speed = app_retract_spd;
            for (i = 0; i < lst.Count && error == 0; i++)
            {
                Plane tp_pln = new Plane();

                if (brnch < TreePln.Branches.Count)
                {
                    tp_pln = TreePln.get_DataItem(TreePln.get_Path(brnch), pln_idx).Value;
                }

                // Change termination type for approach/repose to FINE
                // and CNT for TP
                switch (state)
                {
                    case state.App_Rep:

                        if (lst[i].Origin.DistanceTo(tp_pln.Origin) < 1)
                        {
                            // Toolpath encountered, change state
                            state = state.Tp;

                            speed = tp_speed;

                            pln_idx++;
                        }
                        else // It's an App/Repose
                        {
                            // Move at higher speed (app_retract_speed) ONLY between approach and repose poses //
                            if (switch_spd)
                            {
                                speed = app_retract_spd;
                                switch_spd = false;
                            }
                            else
                            {
                                switch_spd = true;
                            }

                        }
                        break;

                    case state.Tp:
                        if (pln_idx == TreePln.Branches[brnch].Count - 1)
                        {
                            // Last  toolpath pose encountered
                            state = state.App_Rep;
                            term_type = "FINE";

                            pln_idx = 0;
                            switch_spd = false;

                            if (brnch < TreePln.Branches.Count)
                            {
                                brnch++;
                            }

                        }
                        else
                        {
                            term_type = "CNT85 ACC65";

                            if (lst[i].DistanceTo(tp_pln.Origin) < 1)
                            {
                                pln_idx++;
                            }
                            else
                            {
                                error = 1;
                            }
                        }

                        break;

                    default:
                        break;
                }

                file.WriteLine("{0,4:D}:{1} P[{2:D}] {3:D}{4} {5} ;",
                LineCount++, motion_str, PositionCount++, speed, speed_unit, term_type);
            }

            file.WriteLine("\n/POS");

            PositionCount = 1;
            
            file.WriteLine("P[{0}]{{", PositionCount++.ToString());
            file.WriteLine("{0,7}", "GP1:");
            file.WriteLine("        UF : 1, UT : 1,");
            file.WriteLine("{0,10}={1,9:F2} deg,{2,9}={3,9:F2} deg,{4,9}={5,9:F2} deg,", "J1", 0, "J2", 0, "J3", 0);
            file.WriteLine("{0,10}={1,9:F2} deg,{2,9}={3,9:F2} deg,{4,9}={5,9:F2} deg", "J4", 0, "J5", 0, "J6", 0);
            file.WriteLine("};");

            for (i = 0; i < lst.Count; i++)
            {
                if (motion_type == intr_type.Linear)
                {
                    plane = lst[i];

                    orient.Find_WPR(plane);

                    file.WriteLine("P[{0}]{{", PositionCount++.ToString());
                    file.WriteLine("{0,7}", "GP1:");
                    file.WriteLine("        UF : 1, UT : 1,         CONFIG : 'N U T, 0, 0, 0',");
                    file.WriteLine("{0,9} ={1,9:F2}  mm,{2,8} ={3,9:F2}  mm,{4,8} ={5,9:F2}  mm,",
                        "X", lst[i].OriginX * unitScale, "Y", lst[i].OriginY * unitScale, "Z", lst[i].OriginZ * unitScale);
                    file.WriteLine("{0,9} ={1,9:F2} deg,{2,8} ={3,9:F2} deg,{4,8} ={5,9:F2} deg",
                        "W", Rhino.RhinoMath.ToDegrees(orient.WPR.W),
                        "P", Rhino.RhinoMath.ToDegrees(orient.WPR.P),
                        "R", Rhino.RhinoMath.ToDegrees(orient.WPR.R));
                    file.WriteLine("};");
                }
                else
                {
                    file.WriteLine("P[{0}]{{", PositionCount++.ToString());
                    file.WriteLine("{0,7}", "GP1:");
                    file.WriteLine("        UF : 1, UT : 1, ");
                    file.WriteLine("{0,10}={1,9:F2} deg,{2,9}={3,9:F2} deg,{4,9}={5,9:F2} deg,",
                        "J1", TreeAng.get_DataItem(TreeAng.get_Path(0), i).Value * 180/Math.PI,
                        "J2", TreeAng.get_DataItem(TreeAng.get_Path(1), i).Value * 180 / Math.PI,
                        "J3", (TreeAng.get_DataItem(TreeAng.get_Path(2), i).Value - TreeAng.get_DataItem(TreeAng.get_Path(1), i).Value) * 180 / Math.PI);
                    file.WriteLine("{0,10}={1,9:F2} deg,{2,9}={3,9:F2} deg,{4,9}={5,9:F2} deg",
                        "J4", TreeAng.get_DataItem(TreeAng.get_Path(3), i).Value * 180 / Math.PI,
                        "J5", TreeAng.get_DataItem(TreeAng.get_Path(4), i).Value * 180 / Math.PI,
                        "J6", TreeAng.get_DataItem(TreeAng.get_Path(5), i).Value * 180 / Math.PI);
                    file.WriteLine("};");
                }
            }
            file.WriteLine("/END");

            file.Flush();
            file.Close();
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
                return Properties.Resources.FPostIcon;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("{da339d5f-5d85-47bf-8e8a-93a0d267f684}"); }
        }
    }

}

namespace Robot_Math
{  

    public class Robot_Orientation
    {
        /// <summary>
        /// The struct WPR  represents orientation obtained by rotating about the
        /// x, y and z axes w.r.t a fixed reference frame (like the robot base).
        /// </summary>
        public struct t_WPR
        {
            public double W;
            public double P;
            public double R;
        };

        public t_WPR WPR;


        public Robot_Orientation()
        {
            WPR.W = new double();
            WPR.P = new double();
            WPR.R = new double();
        }

        /// <summary>
        /// Updates WPR representation of orientation of the given plane.
        /// </summary>
        public void Find_WPR(Plane plane)
        {
            double temp;

            WPR.W = Math.Atan2(plane.YAxis.Z, plane.ZAxis.Z);
            temp = Math.Pow(plane.YAxis.Z, 2) + Math.Pow(plane.ZAxis.Z, 2);
            WPR.P = Math.Atan2(plane.XAxis.Z, Math.Pow(temp, 0.5));
            WPR.R = Math.Atan2(plane.XAxis.Y, plane.XAxis.X);
        }
    }
}