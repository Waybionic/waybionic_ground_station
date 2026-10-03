// Dumps a SolidWorks assembly for build_arm_description.py: component tree,
// transforms, mates, colors, and tessellated solid bodies. Every document is
// opened read-only and nothing is saved.
//
// Build on Windows with SolidWorks installed (interop types are embedded):
//   set SW=C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist
//   C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe /nologo /platform:x64
//     /out:export_solidworks_assembly.exe /link:"%SW%\SolidWorks.Interop.sldworks.dll"
//     /reference:System.Web.Extensions.dll export_solidworks_assembly.cs
//
// Usage:
//   export_solidworks_assembly.exe <assembly.SLDASM> <output.json>
//     [--substitute "<component name>=<part.SLDPRT>"] [--moves <moves.json>]
//     [--unsuppress "<component name>"] [--snapshot <isometric.png>] [--no-geometry]
//     [--chord <m>] [--angle <rad>]
// Geometry is written next to the JSON as <output>.bin (float32 xyz vertices
// and int32 triangle indices per face, both in part coordinates).
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using System.Web.Script.Serialization;
using SolidWorks.Interop.sldworks;

static class Program
{
    const int DocumentPart = 1;
    const int DocumentAssembly = 2;
    const int OpenSilentReadOnly = 1 | 2;
    const int SolidBody = 0;
    const int ThisConfiguration = 1;

    static SldWorks app;
    static BinaryWriter geometry;
    static readonly List<double[]> palette = new List<double[]>();
    static readonly List<string> unsuppress = new List<string>();
    static double chord = 0.0005;
    static double angle = 0.5236;

    [STAThread]
    static int Main(string[] args)
    {
        var positional = new List<string>();
        var substitutes = new Dictionary<string, string>();
        string movesPath = null;
        string snapshotPath = null;
        bool withGeometry = true;
        for (int i = 0; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "--substitute":
                    string[] pair = args[++i].Split(new[] { '=' }, 2);
                    substitutes[pair[0]] = Path.GetFullPath(pair[1]);
                    break;
                case "--moves": movesPath = Path.GetFullPath(args[++i]); break;
                case "--unsuppress": unsuppress.Add(args[++i]); break;
                case "--snapshot": snapshotPath = Path.GetFullPath(args[++i]); break;
                case "--no-geometry": withGeometry = false; break;
                case "--chord": chord = double.Parse(args[++i], CultureInfo.InvariantCulture); break;
                case "--angle": angle = double.Parse(args[++i], CultureInfo.InvariantCulture); break;
                default: positional.Add(args[i]); break;
            }
        }
        if (positional.Count != 2)
        {
            Console.Error.WriteLine("usage: export_solidworks_assembly <assembly.SLDASM> <output.json> "
                + "[--substitute name=part] [--moves moves.json] [--unsuppress name] [--snapshot png] "
                + "[--no-geometry] [--chord m] [--angle rad]");
            return 2;
        }

        string assemblyPath = Path.GetFullPath(positional[0]);
        string outputPath = Path.GetFullPath(positional[1]);
        var watch = Stopwatch.StartNew();
        try
        {
            app = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
            app.Visible = false;
            var result = new Dictionary<string, object>();
            using (geometry = new BinaryWriter(File.Create(Path.ChangeExtension(outputPath, ".bin"))))
            {
                ModelDoc2 document = Open(assemblyPath, DocumentAssembly, result);
                var assembly = (AssemblyDoc)document;
                result["assembly"] = assemblyPath;
                result["solidworks"] = app.RevisionNumber();
                result["configuration"] = document.ConfigurationManager.ActiveConfiguration.Name;
                result["tolerances"] = new Dictionary<string, object> { { "chord", chord }, { "angle", angle } };
                var components = new List<object>();
                foreach (Component2 component in Components(assembly))
                {
                    components.Add(DescribeComponent(component, substitutes, withGeometry));
                }
                result["components"] = components;
                result["mates"] = CollectAllMates(document, assembly);
                result["palette"] = palette;
                if (snapshotPath != null)
                {
                    app.Visible = true;
                    app.FrameState = 1;
                    result["unsuppressed"] = Restore(document, assembly);
                    document.ShowNamedView2("*Isometric", 7);
                    document.ViewZoomtofit2();
                    document.GraphicsRedraw2();
                    int errors = 0;
                    int warnings = 0;
                    result["snapshot"] = document.Extension.SaveAs(
                        snapshotPath, 0, 1, null, ref errors, ref warnings);
                }
                if (movesPath != null)
                {
                    result["moves"] = RunMoves(assemblyPath, movesPath, document);
                }
            }

            var serializer = new JavaScriptSerializer { MaxJsonLength = int.MaxValue };
            File.WriteAllText(outputPath, serializer.Serialize(result));
            Console.WriteLine("Wrote " + outputPath + " in " + watch.Elapsed.TotalSeconds.ToString("F1") + " s");
            return 0;
        }
        catch (Exception error)
        {
            Console.Error.WriteLine(error);
            return 1;
        }
        finally
        {
            if (app != null)
            {
                try { app.CloseAllDocuments(true); } catch (COMException) { }
                try { app.ExitApp(); } catch (COMException) { }
                Marshal.FinalReleaseComObject(app);
            }
        }
    }

    static ModelDoc2 Open(string path, int type, Dictionary<string, object> report)
    {
        int errors = 0;
        int warnings = 0;
        var document = app.OpenDoc6(path, type, OpenSilentReadOnly, "", ref errors, ref warnings) as ModelDoc2;
        if (document == null)
        {
            throw new InvalidOperationException(
                "OpenDoc6 failed for " + path + ": errors=" + errors + " warnings=" + warnings);
        }
        int resolveStatus = type == DocumentAssembly
            ? ((AssemblyDoc)document).ResolveAllLightWeightComponents(false) : 0;
        if (report != null)
        {
            report["openErrors"] = errors;
            report["openWarnings"] = warnings;
            report["resolveStatus"] = resolveStatus;
        }
        return document;
    }

    static IEnumerable<Component2> Components(AssemblyDoc assembly)
    {
        foreach (object item in (object[])assembly.GetComponents(false))
        {
            yield return (Component2)item;
        }
    }

    static double[] TransformOf(Component2 component)
    {
        var transform = component.Transform2;
        return transform == null ? null : (double[])transform.ArrayData;
    }

    static Dictionary<string, object> DescribeComponent(
        Component2 component, Dictionary<string, string> substitutes, bool withGeometry)
    {
        var watch = Stopwatch.StartNew();
        var data = new Dictionary<string, object>();
        var parent = component.GetParent() as Component2;
        data["name"] = component.Name2;
        data["path"] = component.GetPathName();
        data["parent"] = parent == null ? null : parent.Name2;
        data["configuration"] = component.ReferencedConfiguration;
        data["suppressed"] = component.IsSuppressed();
        data["hidden"] = component.IsHidden(true);
        data["fixed"] = component.IsFixed();
        data["transform"] = TransformOf(component);
        data["box"] = Try(() => component.GetBox(false, false));

        var model = component.GetModelDoc2() as ModelDoc2;
        data["documentType"] = model == null ? 0 : model.GetType();
        data["componentColor"] = component.HasMaterialPropertyValues()
            ? ColorIndex(component.MaterialPropertyValues as double[]) : -1;
        if (!withGeometry)
        {
            return data;
        }

        string substitute;
        List<object> faces = null;
        if (substitutes.TryGetValue(component.Name2, out substitute))
        {
            ModelDoc2 part = Open(substitute, DocumentPart, null);
            data["substitute"] = substitute;
            data["partColor"] = ColorIndex(part.MaterialPropertyValues as double[]);
            data["partBox"] = Try(() => ((PartDoc)part).GetPartBox(true));
            faces = TessellateBodies(((PartDoc)part).GetBodies2(SolidBody, true) as object[], true);
            app.QuitDoc(part.GetTitle());
        }
        else if (model != null && model.GetType() == DocumentPart
            && !component.IsSuppressed() && !component.IsHidden(true))
        {
            object bodyInfo;
            data["partColor"] = ColorIndex(model.MaterialPropertyValues as double[]);
            data["partBox"] = Try(() => ((PartDoc)model).GetPartBox(true));
            faces = TessellateBodies(component.GetBodies3(SolidBody, out bodyInfo) as object[], false);
        }
        if (faces != null)
        {
            data["faces"] = faces;
            long triangles = 0;
            foreach (Dictionary<string, object> face in faces)
            {
                triangles += (int)face["nt"];
            }
            Console.WriteLine(component.Name2 + ": " + faces.Count + " faces, " + triangles
                + " triangles, " + watch.ElapsedMilliseconds + " ms");
        }
        return data;
    }

    static List<object> TessellateBodies(object[] bodies, bool recordCylinders)
    {
        var faces = new List<object>();
        if (bodies == null)
        {
            return faces;
        }
        for (int bodyIndex = 0; bodyIndex < bodies.Length; bodyIndex++)
        {
            var body = (Body2)bodies[bodyIndex];
            var tessellation = (Tessellation)body.GetTessellation(null);
            tessellation.NeedFaceFacetMap = true;
            tessellation.NeedVertexNormal = false;
            tessellation.NeedVertexParams = false;
            tessellation.NeedEdgeFinMap = false;
            tessellation.NeedErrorList = false;
            tessellation.ImprovedQuality = true;
            tessellation.MatchType = 0;
            tessellation.CurveChordTolerance = chord;
            tessellation.CurveChordAngleTolerance = angle;
            tessellation.SurfacePlaneTolerance = chord;
            tessellation.SurfacePlaneAngleTolerance = angle;
            double[][] points = null;
            if (tessellation.Tessellate())
            {
                points = new double[tessellation.GetVertexCount()][];
                for (int index = 0; index < points.Length; index++)
                {
                    points[index] = (double[])tessellation.GetVertexPoint(index);
                }
            }

            int bodyColor = body.HasMaterialPropertyValues()
                ? ColorIndex(body.MaterialPropertyValues2 as double[]) : -1;
            foreach (object item in (object[])body.GetFaces())
            {
                var face = (Face2)item;
                Dictionary<string, object> record = points != null
                    ? FromTessellation(tessellation, face, points) : FromDisplay(face);
                var feature = face.GetFeature() as Feature;
                record["body"] = bodyIndex;
                record["faceColor"] = face.HasMaterialPropertyValues()
                    ? ColorIndex(face.MaterialPropertyValues as double[]) : -1;
                record["featureColor"] = feature != null && feature.HasMaterialPropertyValues()
                    ? ColorIndex(feature.GetMaterialPropertyValues2(ThisConfiguration, null) as double[]) : -1;
                record["bodyColor"] = bodyColor;
                if (recordCylinders)
                {
                    var surface = face.GetSurface() as Surface;
                    if (surface != null && surface.IsCylinder())
                    {
                        record["cylinder"] = surface.CylinderParams;
                    }
                }
                faces.Add(record);
            }
        }
        return faces;
    }

    static Dictionary<string, object> FromTessellation(Tessellation tessellation, Face2 face, double[][] points)
    {
        var local = new Dictionary<int, int>();
        var vertices = new List<double[]>();
        var triangles = new List<int>();
        var facets = tessellation.GetFaceFacets(face) as int[];
        if (facets != null)
        {
            foreach (int facet in facets)
            {
                var fins = (int[])tessellation.GetFacetFins(facet);
                var first = (int[])tessellation.GetFinVertices(fins[0]);
                var second = (int[])tessellation.GetFinVertices(fins[1]);
                int third = second[0] != first[0] && second[0] != first[1] ? second[0] : second[1];
                foreach (int corner in new[] { first[0], first[1], third })
                {
                    int index;
                    if (!local.TryGetValue(corner, out index))
                    {
                        index = vertices.Count;
                        local[corner] = index;
                        vertices.Add(points[corner]);
                    }
                    triangles.Add(index);
                }
            }
        }
        return Write(vertices, triangles, "brep");
    }

    static Dictionary<string, object> FromDisplay(Face2 face)
    {
        var local = new Dictionary<string, int>();
        var vertices = new List<double[]>();
        var triangles = new List<int>();
        var values = face.GetTessTriangles(true) as float[];
        for (int offset = 0; values != null && offset + 2 < values.Length; offset += 3)
        {
            var point = new double[] { values[offset], values[offset + 1], values[offset + 2] };
            string key = point[0].ToString("R") + "," + point[1].ToString("R") + "," + point[2].ToString("R");
            int index;
            if (!local.TryGetValue(key, out index))
            {
                index = vertices.Count;
                local[key] = index;
                vertices.Add(point);
            }
            triangles.Add(index);
        }
        return Write(vertices, triangles, "display");
    }

    static Dictionary<string, object> Write(List<double[]> vertices, List<int> triangles, string source)
    {
        var record = new Dictionary<string, object>();
        record["source"] = source;
        record["v"] = geometry.BaseStream.Position;
        record["nv"] = vertices.Count;
        foreach (double[] point in vertices)
        {
            geometry.Write((float)point[0]);
            geometry.Write((float)point[1]);
            geometry.Write((float)point[2]);
        }
        record["t"] = geometry.BaseStream.Position;
        record["nt"] = triangles.Count / 3;
        foreach (int index in triangles)
        {
            geometry.Write(index);
        }
        return record;
    }

    static int ColorIndex(double[] values)
    {
        if (values == null || values.Length < 3 || values[0] < 0)
        {
            return -1;
        }
        for (int index = 0; index < palette.Count; index++)
        {
            bool same = true;
            for (int channel = 0; channel < values.Length && same; channel++)
            {
                same = Math.Abs(palette[index][channel] - values[channel]) < 1e-6;
            }
            if (same)
            {
                return index;
            }
        }
        palette.Add(values);
        return palette.Count - 1;
    }

    static List<object> CollectAllMates(ModelDoc2 document, AssemblyDoc assembly)
    {
        var mates = new List<object>();
        CollectMates(document, "", mates);
        foreach (Component2 component in Components(assembly))
        {
            var model = component.GetModelDoc2() as ModelDoc2;
            if (model != null && model.GetType() == DocumentAssembly && !component.IsSuppressed())
            {
                CollectMates(model, component.Name2, mates);
            }
        }
        return mates;
    }

    static void CollectMates(ModelDoc2 owner, string ownerName, List<object> mates)
    {
        var feature = owner.FirstFeature() as Feature;
        while (feature != null)
        {
            if (feature.GetTypeName2() == "MateGroup")
            {
                var subFeature = feature.GetFirstSubFeature() as Feature;
                while (subFeature != null)
                {
                    var mate = subFeature.GetSpecificFeature2() as Mate2;
                    if (mate != null)
                    {
                        mates.Add(DescribeMate(subFeature, mate, ownerName));
                    }
                    subFeature = subFeature.GetNextSubFeature() as Feature;
                }
            }
            feature = feature.GetNextFeature() as Feature;
        }
    }

    static Dictionary<string, object> DescribeMate(Feature feature, Mate2 mate, string ownerName)
    {
        bool isWarning;
        var data = new Dictionary<string, object>();
        data["owner"] = ownerName;
        data["name"] = feature.Name;
        data["type"] = mate.Type;
        data["suppressed"] = feature.IsSuppressed();
        data["errorCode"] = feature.GetErrorCode2(out isWarning);
        data["errorIsWarning"] = isWarning;
        data["alignment"] = mate.Alignment;
        data["flipped"] = mate.Flipped;
        var entities = new List<object>();
        for (int index = 0; index < mate.GetMateEntityCount(); index++)
        {
            var entity = mate.MateEntity(index);
            var component = entity.ReferenceComponent;
            entities.Add(new Dictionary<string, object>
            {
                { "component", component == null ? null : component.Name2 },
                { "referenceType", entity.ReferenceType2 },
                { "parameters", entity.EntityParams },
            });
        }
        data["entities"] = entities;
        return data;
    }

    static List<object> RunMoves(string assemblyPath, string movesPath, ModelDoc2 document)
    {
        var serializer = new JavaScriptSerializer();
        var moves = serializer.Deserialize<List<Dictionary<string, object>>>(File.ReadAllText(movesPath));
        var math = (MathUtility)app.GetMathUtility();
        var results = new List<object>();
        foreach (Dictionary<string, object> move in moves)
        {
            app.QuitDoc(document.GetTitle());
            document = Open(assemblyPath, DocumentAssembly, null);
            var assembly = (AssemblyDoc)document;
            var restored = Restore(document, assembly);
            var before = MateErrors(document, assembly);
            var component = assembly.GetComponentByName((string)move["component"]);
            if (component == null)
            {
                throw new InvalidOperationException("Move component not found: " + move["component"]);
            }
            int steps = move.ContainsKey("steps") ? Convert.ToInt32(move["steps"]) : 1;
            var rotation = (MathTransform)math.CreateTransformRotateAxis(
                math.CreatePoint(Doubles(move["point"])), math.CreateVector(Doubles(move["axis"])),
                Convert.ToDouble(move["angle"], CultureInfo.InvariantCulture) / steps);
            bool solved = true;
            string method = move.ContainsKey("method") ? (string)move["method"] : "solve";
            if (method.StartsWith("drag"))
            {
                // drag:<transform type>:<drag mode>
                string[] parts = method.Split(':');
                var drag = (DragOperator)assembly.GetDragOperator();
                drag.AddComponent(component, false);
                drag.CollisionDetectionEnabled = false;
                drag.DynamicClearanceEnabled = false;
                drag.UseAbsoluteTransform = false;
                drag.TransformType = short.Parse(parts[1]);
                drag.DragMode = short.Parse(parts[2]);
                solved &= drag.BeginDrag();
                for (int step = 0; step < steps; step++)
                {
                    solved &= drag.Drag(rotation);
                }
                solved &= drag.EndDrag();
            }
            for (int step = 0; step < steps && method == "solve"; step++)
            {
                var target = (MathTransform)component.Transform2.Multiply(rotation);
                solved &= component.SetTransformAndSolve2(target);
            }
            var transforms = new Dictionary<string, object>();
            foreach (Component2 item in Components(assembly))
            {
                transforms[item.Name2] = TransformOf(item);
            }
            results.Add(new Dictionary<string, object>
            {
                { "name", move["name"] }, { "component", move["component"] },
                { "angle", move["angle"] }, { "solved", solved }, { "transforms", transforms },
                { "unsuppressed", restored }, { "mateErrorsBefore", before },
                { "mateErrorsAfter", MateErrors(document, assembly) },
            });
            Console.WriteLine("Move " + move["name"] + ": solved=" + solved);
        }
        return results;
    }

    static Dictionary<string, object> Restore(ModelDoc2 document, AssemblyDoc assembly)
    {
        // In-memory only: resolve components that were saved suppressed.
        var restored = new Dictionary<string, object>();
        foreach (string name in unsuppress)
        {
            Component2 hidden = null;
            foreach (Component2 item in Components(assembly))
            {
                if (item.Name2 == name) hidden = item;
            }
            restored[name] = hidden == null ? -1 : hidden.SetSuppression2(2);
        }
        if (unsuppress.Count > 0)
        {
            document.ForceRebuild3(false);
        }
        return restored;
    }

    static List<object> MateErrors(ModelDoc2 document, AssemblyDoc assembly)
    {
        var errors = new List<object>();
        foreach (Dictionary<string, object> mate in CollectAllMates(document, assembly))
        {
            if ((int)mate["errorCode"] != 0 && !(bool)mate["suppressed"])
            {
                errors.Add(mate["owner"] + ":" + mate["name"] + "=" + mate["errorCode"]);
            }
        }
        return errors;
    }

    static double[] Doubles(object value)
    {
        var items = (System.Collections.ArrayList)value;
        var result = new double[items.Count];
        for (int index = 0; index < items.Count; index++)
        {
            result[index] = Convert.ToDouble(items[index], CultureInfo.InvariantCulture);
        }
        return result;
    }

    static object Try(Func<object> read)
    {
        try { return read(); }
        catch (COMException) { return null; }
        catch (InvalidCastException) { return null; }
    }
}
