#include "VTK_render.h"

// Define interaction style
class InteractorStyle : public vtkInteractorStyleRubberBandPick
{
public:
	static InteractorStyle* New();
	vtkTypeMacro(InteractorStyle, vtkInteractorStyleRubberBandPick);

	InteractorStyle()
	{
		this->SelectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
		this->SelectedActor = vtkSmartPointer<vtkActor>::New();
		this->SelectedActor->SetMapper(SelectedMapper);
	}

	virtual void OnLeftButtonUp()
	{
		// Forward events
		vtkInteractorStyleRubberBandPick::OnLeftButtonUp();

		vtkPlanes* frustum = static_cast<vtkAreaPicker*>(this->GetInteractor()->GetPicker())->GetFrustum();

		vtkSmartPointer<vtkExtractGeometry> extractGeometry =
			vtkSmartPointer<vtkExtractGeometry>::New();
		extractGeometry->SetImplicitFunction(frustum);
#if VTK_MAJOR_VERSION <= 5
		extractGeometry->SetInput(this->Points);
#else
		extractGeometry->SetInputData(this->Points);
#endif
		extractGeometry->Update();

		vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
			vtkSmartPointer<vtkVertexGlyphFilter>::New();
		glyphFilter->SetInputConnection(extractGeometry->GetOutputPort());
		glyphFilter->Update();

		vtkPolyData* selected = glyphFilter->GetOutput();
		std::cout << "Selected " << selected->GetNumberOfPoints() << " points." << std::endl;
		std::cout << "Selected " << selected->GetNumberOfCells() << " cells." << std::endl;
#if VTK_MAJOR_VERSION <= 5
		this->SelectedMapper->SetInput(selected);
#else
		this->SelectedMapper->SetInputData(selected);
#endif
		this->SelectedMapper->ScalarVisibilityOff();

		vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(selected->GetPointData()->GetArray("OriginalIds"));
		for (vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
		{
			std::cout << "Id " << i << " : " << ids->GetValue(i) << std::endl;
		}

		this->SelectedActor->GetProperty()->SetColor(1.0, 0.0, 0.0); //(R,G,B)
		this->SelectedActor->GetProperty()->SetPointSize(3);

		this->CurrentRenderer->AddActor(SelectedActor);
		this->GetInteractor()->GetRenderWindow()->Render();
		this->HighlightProp(NULL);
	}

	void SetPoints(vtkSmartPointer<vtkPolyData> points) { this->Points = points; }
private:
	vtkSmartPointer<vtkPolyData> Points;
	vtkSmartPointer<vtkActor> SelectedActor;
	vtkSmartPointer<vtkDataSetMapper> SelectedMapper;

};
vtkStandardNewMacro(InteractorStyle);

void MeshRender(orth::MeshModel &mm,float size_)
{
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
		vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	//orth::PointCloudD Vertices = mm.P;
	vector<double> Vertices(mm.P.size() * 3);
	memcpy(Vertices.data(), mm.P[0].data(), sizeof(double)*mm.P.size() * 3);

	double min_dis = Vertices[1];
	double max_dis = Vertices[1];
	for (int point_index = 1; point_index < Vertices.size() / 3; point_index++)
	{
		if (Vertices[3 * point_index + 1]>max_dis)
		{
			max_dis = Vertices[3 * point_index + 1];
		}
		if (Vertices[3 * point_index + 1]<min_dis)
		{
			min_dis = Vertices[3 * point_index + 1];
		}
	}
	double l = max_dis - min_dis;
	for (int point_index = 0; point_index < Vertices.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices[3 * point_index + 0], Vertices[3 * point_index + 1], Vertices[3 * point_index + 2]);
		unsigned char color[3] = { (Vertices[3 * point_index + 1] - min_dis)*255.0 / l,255 - (Vertices[3 * point_index + 1] - min_dis)*255.0 / l,127 };
		colors->InsertNextTupleValue(color);
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());

	polydata->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(30, 30, 30);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	// Visualization

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(size_);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();

	/*----------------------------- camera 1 --------------------------*/
	//vtkSmartPointer<vtkCamera> camera1 =
	//	vtkSmartPointer<vtkCamera>::New();
	//camera1->SetPosition(300,50,-300);
	//camera1->SetFocalPoint(50,50,-10);
	//renderer->SetActiveCamera(camera1);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkAreaPicker> areaPicker =
		vtkSmartPointer<vtkAreaPicker>::New();
	renderWindowInteractor->SetPicker(areaPicker);

	renderer->AddActor(axes);
	renderer->AddActor(actor);
	renderer->SetBackground(namedColors->GetColor3d("White").GetData());
	renderWindow->Render();

	renderWindowInteractor->Start();

}

void MeshRender(orth::MaxillaryTeeth &mm)

{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	// Setup colors
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
	if (!(mm.teeths.size()))
	{
		for (size_t face_num = 0; face_num < mm.F.size(); face_num++)
		{
			points->InsertNextPoint(mm.P[mm.F[face_num].x].x, mm.P[mm.F[face_num].x].y, mm.P[mm.F[face_num].x].z);
			points->InsertNextPoint(mm.P[mm.F[face_num].y].x, mm.P[mm.F[face_num].y].y, mm.P[mm.F[face_num].y].z);
			points->InsertNextPoint(mm.P[mm.F[face_num].z].x, mm.P[mm.F[face_num].z].y, mm.P[mm.F[face_num].z].z);
			vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
			polygon->GetPointIds()->SetNumberOfIds(3); //make a triple
			polygon->GetPointIds()->SetId(0, face_num * 3);
			polygon->GetPointIds()->SetId(1, face_num * 3 + 1);
			polygon->GetPointIds()->SetId(2, face_num * 3 + 2);
			polygons->InsertNextCell(polygon);
			unsigned char color[3] = { 180,180,180 };
			colors->InsertNextTupleValue(color);
			colors->InsertNextTupleValue(color);
			colors->InsertNextTupleValue(color);
		}
	}
	else
	{
		int point_id = 0;
		for (size_t teeth_index = 0; teeth_index < mm.teeths.size(); teeth_index++)
		{
			for (size_t face_num = 0; face_num < mm.teeths[teeth_index].F.size(); face_num++)
			{
				points->InsertNextPoint(mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].x].x, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].x].y, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].x].z);
				points->InsertNextPoint(mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].y].x, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].y].y, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].y].z);
				points->InsertNextPoint(mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].z].x, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].z].y, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].z].z);
				vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
				polygon->GetPointIds()->SetNumberOfIds(3); //make a triple
				polygon->GetPointIds()->SetId(0, point_id++);
				polygon->GetPointIds()->SetId(1, point_id++);
				polygon->GetPointIds()->SetId(2, point_id++);
				polygons->InsertNextCell(polygon);

				/*
				198 226 255
				159 121 238
				0   139 139
				238 118 0
				32  178 170
				255 236 139
				0   255 255
				112 128 144
				84  255 159
				255 255 0
				178 34  34
				148 0   211
				238 0   0
				238 213 210
				99  184 255
				144 238 144
				232 232 232
				139 0   139
				0   0   139
				139 0   0
				238 64  20
				255 165 0
				34  139 34
				255 215 0
				127 255 212
				255 246 143
				105 105 105
				110 139 61
				0   206 209
				139 69  19
				159 121 238
				255 250 250
				*/

				unsigned char color[3];
				color[0] = mm.teeths[teeth_index].C[mm.teeths[teeth_index].F[face_num].x].x;
				color[1] = mm.teeths[teeth_index].C[mm.teeths[teeth_index].F[face_num].x].y;
				color[2] = mm.teeths[teeth_index].C[mm.teeths[teeth_index].F[face_num].x].z;

				//switch (teeth_index)
				//{
				//case -1: {color[0] = 180; color[1] = 180; color[2] = 180; break; }
				//case 0:{color[0] = 198; color[1] = 226; color[2] = 255; break; }// 
				//case 1: {color[0] = 159; color[1] = 121; color[2] = 238; break; }
				//case 2: {color[0] = 0; color[1] = 139; color[2] = 139; break; }
				//case 3: {color[0] = 238; color[1] = 118; color[2] = 0; break; }
				//case 4: {color[0] = 32; color[1] = 178; color[2] = 170; break; }
				//case 5: {color[0] = 255; color[1] = 236; color[2] = 139; break; }
				//case 6: {color[0] = 0; color[1] = 0; color[2] = 0; break; } //{color[0] = 0; color[1] = 255; color[2] = 255; break; }
				//case 7: {color[0] = 112; color[1] = 128; color[2] = 144; break; }
				//case 8: {color[0] = 84; color[1] = 255; color[2] = 159; break; }
				//case 9: {color[0] = 255; color[1] = 255; color[2] = 0; break; }
				//case 10: {color[0] = 178; color[1] = 34; color[2] = 34; break; }
				//case 11: {color[0] = 148; color[1] = 0; color[2] = 211; break; }
				//case 12: {color[0] = 238; color[1] = 0; color[2] = 0; break; }
				//case 13: {color[0] = 238; color[1] = 213; color[2] = 210; break; }
				//case 14: {color[0] = 99; color[1] = 184; color[2] = 255; break; }
				//case 15: {color[0] = 144; color[1] = 238; color[2] = 144; break; }
				//case 16: {color[0] = 232; color[1] = 232; color[2] = 232; break; }
				//case 17: {color[0] = 139; color[1] = 0; color[2] = 139; break; }
				//case 18: {color[0] = 0; color[1] = 0; color[2] = 139; break; }
				//case 19: {color[0] = 139; color[1] = 0; color[2] = 0; break; }
				//case 20: {color[0] = 238; color[1] = 64; color[2] = 20; break; }
				//case 21: {color[0] = 255; color[1] = 165; color[2] = 0; break; }
				//case 22: {color[0] = 34; color[1] = 139; color[2] = 34; break; }
				//case 23: {color[0] = 255; color[1] = 215; color[2] = 0; break; }
				//case 24: {color[0] = 127; color[1] = 255; color[2] = 212; break; }
				//case 25: {color[0] = 255; color[1] = 246; color[2] = 143; break; }
				//case 26: {color[0] = 105; color[1] = 105; color[2] = 105; break; }
				//case 27: {color[0] = 110; color[1] = 139; color[2] = 61; break; }
				//case 28: {color[0] = 0; color[1] = 206; color[2] = 209; break; }
				//case 29: {color[0] = 139; color[1] = 69; color[2] = 19; break; }
				//case 30: {color[0] = 159; color[1] = 121; color[2] = 238; break; }
				//case 31: {color[0] = 255; color[1] = 250; color[2] = 250; break; }
				//default:
				//	break;
				//}

				//unsigned char color[3] = { mm.L[mm.F[face_num].x],180,180 };
				colors->InsertNextTupleValue(color);
				colors->InsertNextTupleValue(color);
				colors->InsertNextTupleValue(color);
			}
		}

		//for (size_t face_num = 0; face_num < mm.F.size(); face_num++)
		//{
		//	points->InsertNextPoint(mm.P[mm.F[face_num].x].x, mm.P[mm.F[face_num].x].y, mm.P[mm.F[face_num].x].z);
		//	points->InsertNextPoint(mm.P[mm.F[face_num].y].x, mm.P[mm.F[face_num].y].y, mm.P[mm.F[face_num].y].z);
		//	points->InsertNextPoint(mm.P[mm.F[face_num].z].x, mm.P[mm.F[face_num].z].y, mm.P[mm.F[face_num].z].z);
		//	vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
		//	polygon->GetPointIds()->SetNumberOfIds(3); //make a triple
		//	polygon->GetPointIds()->SetId(0, face_num * 3);
		//	polygon->GetPointIds()->SetId(1, face_num * 3 + 1);
		//	polygon->GetPointIds()->SetId(2, face_num * 3 + 2);
		//	polygons->InsertNextCell(polygon);

		//	/*
		//	198 226 255
		//	159 121 238
		//	0   139 139
		//	238 118 0
		//	32  178 170
		//	255 236 139
		//	0   255 255
		//	112 128 144
		//	84  255 159
		//	255 255 0
		//	178 34  34
		//	148 0   211
		//	238 0   0
		//	238 213 210
		//	99  184 255
		//	144 238 144
		//	232 232 232
		//	139 0   139
		//	0   0   139
		//	139 0   0
		//	238 64  20
		//	255 165 0
		//	34  139 34
		//	255 215 0
		//	127 255 212
		//	255 246 143
		//	105 105 105
		//	110 139 61
		//	0   206 209
		//	139 69  19
		//	159 121 238
		//	255 250 250
		//	*/

		//	unsigned char color[3];
		//	switch ((int)(mm.L[mm.F[face_num].x]))
		//	{
		//	case -1: {color[0] = 180; color[1] = 180; color[2] = 180; break; }
		//	case 0: {color[0] = 198; color[1] = 226; color[2] = 255; break; }
		//	case 1: {color[0] = 159; color[1] = 121; color[2] = 238; break; }
		//	case 2: {color[0] = 0; color[1] = 139; color[2] = 139; break; }
		//	case 3: {color[0] = 238; color[1] = 118; color[2] = 0; break; }
		//	case 4: {color[0] = 32; color[1] = 178; color[2] = 170; break; }
		//	case 5: {color[0] = 255; color[1] = 236; color[2] = 139; break; }
		//	case 6: {color[0] = 0; color[1] = 255; color[2] = 255; break; }
		//	case 7: {color[0] = 112; color[1] = 128; color[2] = 144; break; }
		//	case 8: {color[0] = 84; color[1] = 255; color[2] = 159; break; }
		//	case 9: {color[0] = 255; color[1] = 255; color[2] = 0; break; }
		//	case 10: {color[0] = 178; color[1] = 34; color[2] = 34; break; }
		//	case 11: {color[0] = 148; color[1] = 0; color[2] = 211; break; }
		//	case 12: {color[0] = 238; color[1] = 0; color[2] = 0; break; }
		//	case 13: {color[0] = 238; color[1] = 213; color[2] = 210; break; }
		//	case 14: {color[0] = 99; color[1] = 184; color[2] = 255; break; }
		//	case 15: {color[0] = 144; color[1] = 238; color[2] = 144; break; }
		//	case 16: {color[0] = 232; color[1] = 232; color[2] = 232; break; }
		//	case 17: {color[0] = 139; color[1] = 0; color[2] = 139; break; }
		//	case 18: {color[0] = 0; color[1] = 0; color[2] = 139; break; }
		//	case 19: {color[0] = 139; color[1] = 0; color[2] = 0; break; }
		//	case 20: {color[0] = 238; color[1] = 64; color[2] = 20; break; }
		//	case 21: {color[0] = 255; color[1] = 165; color[2] = 0; break; }
		//	case 22: {color[0] = 34; color[1] = 139; color[2] = 34; break; }
		//	case 23: {color[0] = 255; color[1] = 215; color[2] = 0; break; }
		//	case 24: {color[0] = 127; color[1] = 255; color[2] = 212; break; }
		//	case 25: {color[0] = 255; color[1] = 246; color[2] = 143; break; }
		//	case 26: {color[0] = 105; color[1] = 105; color[2] = 105; break; }
		//	case 27: {color[0] = 110; color[1] = 139; color[2] = 61; break; }
		//	case 28: {color[0] = 0; color[1] = 206; color[2] = 209; break; }
		//	case 29: {color[0] = 139; color[1] = 69; color[2] = 19; break; }
		//	case 30: {color[0] = 159; color[1] = 121; color[2] = 238; break; }
		//	case 31: {color[0] = 255; color[1] = 250; color[2] = 250; break; }
		//	default:
		//		break;
		//	}
		//	
		//	//unsigned char color[3] = { mm.L[mm.F[face_num].x],180,180 };
		//	colors->InsertNextTupleValue(color);
		//	colors->InsertNextTupleValue(color);
		//	colors->InsertNextTupleValue(color);

		//}
	}

	vtkSmartPointer<vtkPolyData> triple = vtkSmartPointer<vtkPolyData>::New();
	triple->SetPoints(points);
	//cout << points->GetReferenceCount() << endl;
	triple->SetPolys(polygons);

	triple->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInput(triple);
#else
	mapper->SetInputData(triple);
#endif

	vtkSmartPointer<vtkActor> texturedQuad =
		vtkSmartPointer<vtkActor>::New();
	texturedQuad->SetMapper(mapper);

	// Visualize the textured plane
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(texturedQuad);



	vtkSmartPointer<vtkNamedColors> colors2 =
		vtkSmartPointer<vtkNamedColors>::New();
	vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
	// Create three points. We will join (Origin and P0) with a red line and (Origin and P1) with a green line
	//if (mm.arch.size()==3)
	//{
	//	double a, b, c, d;
	//	orth::Point3d p1 = mm.arch[0], p2 = mm.arch[1], p3 = mm.arch[2];
	//	a = (p2.y - p3.y) / (p2.x*p2.x-2*p1.x*p2.x-p3.x*p3.x+2*p1.x*p3.x);
	//	b = -2 * p1.x*a;
	//	c = p1.x*p1.x*a + p1.y;
	//	cout << a << " -- " << b << " -- " << c << endl;
	//	double p[3];
	//	for (size_t x_index = -30; x_index < 30; x_index++)
	//	{
	//		p[0] = x_index;
	//		p[1] = a * x_index*x_index + b*x_index + c;
	//		p[2] = 0;
	//		points2->InsertNextPoint(p);
	//	}
	//}

	if (mm.arch.size())
	{
		double p[3];

		for (size_t x_index = 0; x_index < mm.arch.size(); x_index++)
		{
			p[0] = mm.arch[x_index].x;
			p[1] = mm.arch[x_index].y;
			p[2] = mm.arch[x_index].z;
			points2->InsertNextPoint(p);
		}


		vtkSmartPointer<vtkParametricSpline> spline =
			vtkSmartPointer<vtkParametricSpline>::New();
		spline->SetPoints(points2);

		vtkSmartPointer<vtkParametricFunctionSource> functionSource =
			vtkSmartPointer<vtkParametricFunctionSource>::New();
		functionSource->SetParametricFunction(spline);
		functionSource->Update();

		vtkSmartPointer<vtkPolyDataMapper> mapper2 =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper2->SetInputConnection(functionSource->GetOutputPort());

		vtkSmartPointer<vtkActor> actor2 =
			vtkSmartPointer<vtkActor>::New();
		actor2->SetMapper(mapper2);
		actor2->GetProperty()->SetColor(colors2->GetColor3d("Green").GetData());
		actor2->GetProperty()->SetLineWidth(3.0);



		renderer->AddActor(actor2);
	}

	if (mm.division_plane.size()>0)
	{
		for (int plane_index = 0; plane_index < mm.division_plane.size(); plane_index++)
		{
			vtkSmartPointer<vtkNamedColors> colors3 =
				vtkSmartPointer<vtkNamedColors>::New();

			//// Set the background color.
			//std::array<unsigned char, 4> bkg{ { 26, 51, 77, 255 } };
			//colors3->SetColor("BkgColor", bkg.data());


			// Create a plane
			vtkSmartPointer<vtkPlaneSource> planeSource =
				vtkSmartPointer<vtkPlaneSource>::New();
			planeSource->SetCenter(mm.division_plane[plane_index].Center.x, mm.division_plane[plane_index].Center.y, mm.division_plane[plane_index].Center.z);
			planeSource->SetNormal(mm.division_plane[plane_index].A, mm.division_plane[plane_index].B, mm.division_plane[plane_index].C);
			planeSource->SetResolution(20, 20);

			planeSource->Update();

			vtkPolyData* plane = planeSource->GetOutput();

			// Create a mapper and actor
			vtkSmartPointer<vtkPolyDataMapper> mapper3 =
				vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper3->SetInputData(plane);

			vtkSmartPointer<vtkActor> actor3 =
				vtkSmartPointer<vtkActor>::New();
			actor3->SetMapper(mapper3);
			actor3->GetProperty()->SetColor(colors3->GetColor3d("White").GetData());

			renderer->AddActor(actor3);
		}

	}

	//double origin[3] = { 0.0, 0.0, 0.0 };
	//double p0[3] = { 10.0, 0.0, 0.0 };
	//double p1[3] = { 0.0, 10.0, 0.0 };
	//double p2[3] = { 0.0, 10.0, 20.0 };
	//double p3[3] = { 10.0, 20.0, 30.0 };

	//// Create a vtkPoints object and store the points in it
	//
	//points2->InsertNextPoint(origin);
	//points2->InsertNextPoint(p0);
	//points2->InsertNextPoint(p1);
	//points2->InsertNextPoint(p2);
	//points2->InsertNextPoint(p3);


	renderer->SetBackground(0.4392, 0.5020, 0.5647);
	renderer->ResetCamera();

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindow->Render();

	renderWindowInteractor->Start();
}

void MeshRender(orth::MaxillaryTeeth &mm, orth::PointCloudD &V_query, orth::PointCloudD &V_target)
{
	/*------------------------------------------------- teeth -----------------------------------------------------*/

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	// Setup colors
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");


	vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
	if (!(mm.teeths.size()))
	{
		for (size_t face_num = 0; face_num < mm.F.size(); face_num++)
		{
			points->InsertNextPoint(mm.P[mm.F[face_num].x].x, mm.P[mm.F[face_num].x].y, mm.P[mm.F[face_num].x].z);
			points->InsertNextPoint(mm.P[mm.F[face_num].y].x, mm.P[mm.F[face_num].y].y, mm.P[mm.F[face_num].y].z);
			points->InsertNextPoint(mm.P[mm.F[face_num].z].x, mm.P[mm.F[face_num].z].y, mm.P[mm.F[face_num].z].z);
			vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
			polygon->GetPointIds()->SetNumberOfIds(3); //make a triple
			polygon->GetPointIds()->SetId(0, face_num * 3);
			polygon->GetPointIds()->SetId(1, face_num * 3 + 1);
			polygon->GetPointIds()->SetId(2, face_num * 3 + 2);
			polygons->InsertNextCell(polygon);
			unsigned char color[3] = { 180,180,180 };
			colors->InsertNextTupleValue(color);
			colors->InsertNextTupleValue(color);
			colors->InsertNextTupleValue(color);
		}
	}
	else
	{
		int point_id = 0;
		for (size_t teeth_index = 0; teeth_index < mm.teeths.size(); teeth_index++)
		{
			if (teeth_index == 0 || teeth_index == 15)
			{
				continue;
			}
			for (size_t face_num = 0; face_num < mm.teeths[teeth_index].F.size(); face_num++)
			{

				points->InsertNextPoint(mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].x].x, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].x].y, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].x].z);
				points->InsertNextPoint(mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].y].x, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].y].y, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].y].z);
				points->InsertNextPoint(mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].z].x, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].z].y, mm.teeths[teeth_index].P[mm.teeths[teeth_index].F[face_num].z].z);
				vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
				polygon->GetPointIds()->SetNumberOfIds(3); //make a triple
				polygon->GetPointIds()->SetId(0, point_id++);
				polygon->GetPointIds()->SetId(1, point_id++);
				polygon->GetPointIds()->SetId(2, point_id++);
				polygons->InsertNextCell(polygon);

				/*
				198 226 255
				159 121 238
				0   139 139
				238 118 0
				32  178 170
				255 236 139
				0   255 255
				112 128 144
				84  255 159
				255 255 0
				178 34  34
				148 0   211
				238 0   0
				238 213 210
				99  184 255
				144 238 144
				232 232 232
				139 0   139
				0   0   139
				139 0   0
				238 64  20
				255 165 0
				34  139 34
				255 215 0
				127 255 212
				255 246 143
				105 105 105
				110 139 61
				0   206 209
				139 69  19
				159 121 238
				255 250 250
				*/

				unsigned char color[3];
				color[0] = mm.teeths[teeth_index].C[mm.teeths[teeth_index].F[face_num].x].x;
				color[1] = mm.teeths[teeth_index].C[mm.teeths[teeth_index].F[face_num].x].y;
				color[2] = mm.teeths[teeth_index].C[mm.teeths[teeth_index].F[face_num].x].z;

				//unsigned char color[3];
				//switch (teeth_index)
				//{
				//case -1: {color[0] = 180; color[1] = 180; color[2] = 180; break; }
				//case 0:{color[0] = 198; color[1] = 226; color[2] = 255; break; }
				//case 1: {color[0] = 159; color[1] = 121; color[2] = 238; break; }//{color[0] = 0; color[1] = 0; color[2] = 0; break; } //
				//case 2: {color[0] = 0; color[1] = 139; color[2] = 139; break; }
				//case 3: {color[0] = 238; color[1] = 118; color[2] = 0; break; }
				//case 4: {color[0] = 32; color[1] = 178; color[2] = 170; break; }
				//case 5: {color[0] = 255; color[1] = 236; color[2] = 139; break; }
				//case 6: {color[0] = 0; color[1] = 0; color[2] = 255; break; }
				//case 7: {color[0] = 0; color[1] = 0; color[2] = 0; break; } //{color[0] = 112; color[1] = 128; color[2] = 144; break; }
				//case 8: {color[0] = 84; color[1] = 255; color[2] = 159; break; }
				//case 9: {color[0] = 255; color[1] = 255; color[2] = 0; break; }
				//case 10: {color[0] = 178; color[1] = 34; color[2] = 34; break; }
				//case 11: {color[0] = 148; color[1] = 0; color[2] = 211; break; }
				//case 12: {color[0] = 238; color[1] = 0; color[2] = 0; break; }
				//case 13: {color[0] = 238; color[1] = 213; color[2] = 210; break; }
				//case 14: {color[0] = 99; color[1] = 184; color[2] = 255; break; }
				//case 15: {color[0] = 144; color[1] = 238; color[2] = 144; break; }
				//case 16: {color[0] = 232; color[1] = 232; color[2] = 232; break; }
				//case 17: {color[0] = 139; color[1] = 0; color[2] = 139; break; }
				//case 18: {color[0] = 0; color[1] = 0; color[2] = 139; break; }
				//case 19: {color[0] = 139; color[1] = 0; color[2] = 0; break; }
				//case 20: {color[0] = 238; color[1] = 64; color[2] = 20; break; }
				//case 21: {color[0] = 255; color[1] = 165; color[2] = 0; break; }
				//case 22: {color[0] = 34; color[1] = 139; color[2] = 34; break; }
				//case 23: {color[0] = 255; color[1] = 215; color[2] = 0; break; }
				//case 24: {color[0] = 127; color[1] = 255; color[2] = 212; break; }
				//case 25: {color[0] = 255; color[1] = 246; color[2] = 143; break; }
				//case 26: {color[0] = 105; color[1] = 105; color[2] = 105; break; }
				//case 27: {color[0] = 110; color[1] = 139; color[2] = 61; break; }
				//case 28: {color[0] = 0; color[1] = 206; color[2] = 209; break; }
				//case 29: {color[0] = 139; color[1] = 69; color[2] = 19; break; }
				//case 30: {color[0] = 159; color[1] = 121; color[2] = 238; break; }
				//case 31: {color[0] = 255; color[1] = 250; color[2] = 250; break; }
				//default:
				//	break;
				//}

				//unsigned char color[3] = { mm.L[mm.F[face_num].x],180,180 };
				colors->InsertNextTupleValue(color);
				colors->InsertNextTupleValue(color);
				colors->InsertNextTupleValue(color);
			}
		}



		//for (size_t face_num = 0; face_num < mm.F.size(); face_num++)
		//{
		//	points->InsertNextPoint(mm.P[mm.F[face_num].x].x, mm.P[mm.F[face_num].x].y, mm.P[mm.F[face_num].x].z);
		//	points->InsertNextPoint(mm.P[mm.F[face_num].y].x, mm.P[mm.F[face_num].y].y, mm.P[mm.F[face_num].y].z);
		//	points->InsertNextPoint(mm.P[mm.F[face_num].z].x, mm.P[mm.F[face_num].z].y, mm.P[mm.F[face_num].z].z);
		//	vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
		//	polygon->GetPointIds()->SetNumberOfIds(3); //make a triple
		//	polygon->GetPointIds()->SetId(0, face_num * 3);
		//	polygon->GetPointIds()->SetId(1, face_num * 3 + 1);
		//	polygon->GetPointIds()->SetId(2, face_num * 3 + 2);
		//	polygons->InsertNextCell(polygon);

		//	/*
		//	198 226 255
		//	159 121 238
		//	0   139 139
		//	238 118 0
		//	32  178 170
		//	255 236 139
		//	0   255 255
		//	112 128 144
		//	84  255 159
		//	255 255 0
		//	178 34  34
		//	148 0   211
		//	238 0   0
		//	238 213 210
		//	99  184 255
		//	144 238 144
		//	232 232 232
		//	139 0   139
		//	0   0   139
		//	139 0   0
		//	238 64  20
		//	255 165 0
		//	34  139 34
		//	255 215 0
		//	127 255 212
		//	255 246 143
		//	105 105 105
		//	110 139 61
		//	0   206 209
		//	139 69  19
		//	159 121 238
		//	255 250 250
		//	*/

		//	unsigned char color[3];
		//	switch ((int)(mm.L[mm.F[face_num].x]))
		//	{
		//	case -1: {color[0] = 180; color[1] = 180; color[2] = 180; break; }
		//	case 0: {color[0] = 198; color[1] = 226; color[2] = 255; break; }
		//	case 1: {color[0] = 159; color[1] = 121; color[2] = 238; break; }
		//	case 2: {color[0] = 0; color[1] = 139; color[2] = 139; break; }
		//	case 3: {color[0] = 238; color[1] = 118; color[2] = 0; break; }
		//	case 4: {color[0] = 32; color[1] = 178; color[2] = 170; break; }
		//	case 5: {color[0] = 255; color[1] = 236; color[2] = 139; break; }
		//	case 6: {color[0] = 0; color[1] = 255; color[2] = 255; break; }
		//	case 7: {color[0] = 112; color[1] = 128; color[2] = 144; break; }
		//	case 8: {color[0] = 84; color[1] = 255; color[2] = 159; break; }
		//	case 9: {color[0] = 255; color[1] = 255; color[2] = 0; break; }
		//	case 10: {color[0] = 178; color[1] = 34; color[2] = 34; break; }
		//	case 11: {color[0] = 148; color[1] = 0; color[2] = 211; break; }
		//	case 12: {color[0] = 238; color[1] = 0; color[2] = 0; break; }
		//	case 13: {color[0] = 238; color[1] = 213; color[2] = 210; break; }
		//	case 14: {color[0] = 99; color[1] = 184; color[2] = 255; break; }
		//	case 15: {color[0] = 144; color[1] = 238; color[2] = 144; break; }
		//	case 16: {color[0] = 232; color[1] = 232; color[2] = 232; break; }
		//	case 17: {color[0] = 139; color[1] = 0; color[2] = 139; break; }
		//	case 18: {color[0] = 0; color[1] = 0; color[2] = 139; break; }
		//	case 19: {color[0] = 139; color[1] = 0; color[2] = 0; break; }
		//	case 20: {color[0] = 238; color[1] = 64; color[2] = 20; break; }
		//	case 21: {color[0] = 255; color[1] = 165; color[2] = 0; break; }
		//	case 22: {color[0] = 34; color[1] = 139; color[2] = 34; break; }
		//	case 23: {color[0] = 255; color[1] = 215; color[2] = 0; break; }
		//	case 24: {color[0] = 127; color[1] = 255; color[2] = 212; break; }
		//	case 25: {color[0] = 255; color[1] = 246; color[2] = 143; break; }
		//	case 26: {color[0] = 105; color[1] = 105; color[2] = 105; break; }
		//	case 27: {color[0] = 110; color[1] = 139; color[2] = 61; break; }
		//	case 28: {color[0] = 0; color[1] = 206; color[2] = 209; break; }
		//	case 29: {color[0] = 139; color[1] = 69; color[2] = 19; break; }
		//	case 30: {color[0] = 159; color[1] = 121; color[2] = 238; break; }
		//	case 31: {color[0] = 255; color[1] = 250; color[2] = 250; break; }
		//	default:
		//		break;
		//	}
		//	
		//	//unsigned char color[3] = { mm.L[mm.F[face_num].x],180,180 };
		//	colors->InsertNextTupleValue(color);
		//	colors->InsertNextTupleValue(color);
		//	colors->InsertNextTupleValue(color);

		//}
	}




	vtkSmartPointer<vtkPolyData> triple = vtkSmartPointer<vtkPolyData>::New();
	triple->SetPoints(points);
	//cout << points->GetReferenceCount() << endl;
	triple->SetPolys(polygons);

	triple->GetPointData()->SetScalars(colors);


	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInput(triple);
#else
	mapper->SetInputData(triple);
#endif

	vtkSmartPointer<vtkActor> texturedQuad =
		vtkSmartPointer<vtkActor>::New();
	texturedQuad->SetMapper(mapper);

	// Visualize the textured plane
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(texturedQuad);





	vtkSmartPointer<vtkNamedColors> colors2 =
		vtkSmartPointer<vtkNamedColors>::New();
	vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();

	// Create three points. We will join (Origin and P0) with a red line and (Origin and P1) with a green line
	//if (mm.arch.size()==3)
	//{
	//	double a, b, c, d;
	//	orth::Point3d p1 = mm.arch[0], p2 = mm.arch[1], p3 = mm.arch[2];
	//	a = (p2.y - p3.y) / (p2.x*p2.x-2*p1.x*p2.x-p3.x*p3.x+2*p1.x*p3.x);
	//	b = -2 * p1.x*a;
	//	c = p1.x*p1.x*a + p1.y;
	//	cout << a << " -- " << b << " -- " << c << endl;
	//	double p[3];
	//	for (size_t x_index = -30; x_index < 30; x_index++)
	//	{
	//		p[0] = x_index;
	//		p[1] = a * x_index*x_index + b*x_index + c;
	//		p[2] = 0;
	//		points2->InsertNextPoint(p);
	//	}
	//}

	/*------------------------------------------------- arch ----------------------------------------------------*/

	if (mm.arch.size())
	{
		double p[3];
		for (size_t x_index = 0; x_index < mm.arch.size(); x_index++)
		{
			p[0] = mm.arch[x_index].x;
			p[1] = mm.arch[x_index].y;
			p[2] = mm.arch[x_index].z;
			points2->InsertNextPoint(p);
		}


		vtkSmartPointer<vtkParametricSpline> spline =
			vtkSmartPointer<vtkParametricSpline>::New();
		spline->SetPoints(points2);

		vtkSmartPointer<vtkParametricFunctionSource> functionSource =
			vtkSmartPointer<vtkParametricFunctionSource>::New();
		functionSource->SetParametricFunction(spline);
		functionSource->Update();

		vtkSmartPointer<vtkPolyDataMapper> mapper2 =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper2->SetInputConnection(functionSource->GetOutputPort());

		vtkSmartPointer<vtkActor> actor2 =
			vtkSmartPointer<vtkActor>::New();
		actor2->SetMapper(mapper2);
		actor2->GetProperty()->SetColor(colors2->GetColor3d("Green").GetData());
		actor2->GetProperty()->SetLineWidth(3.0);
		renderer->AddActor(actor2);
	}


	/*------------------------------------------------- feature points -----------------------------------------------------*/

	if (V_query.size()>0)
	{
		vtkSmartPointer<vtkPoints> points3 =
			vtkSmartPointer<vtkPoints>::New();

		// Setup colors
		vtkSmartPointer<vtkNamedColors> namedColors3 =
			vtkSmartPointer<vtkNamedColors>::New();

		vtkSmartPointer<vtkUnsignedCharArray> colors3 =
			vtkSmartPointer<vtkUnsignedCharArray>::New();
		colors3->SetNumberOfComponents(3);
		colors3->SetName("Colors3");

		for (int point_index = 0; point_index < V_query.size(); point_index++)
		{
			points3->InsertNextPoint(V_query[point_index].x, V_query[point_index].y, V_query[point_index].z);
			unsigned char color3[3] = { 255,100,100 };
			colors3->InsertNextTupleValue(color3);
		}

		for (int point_index = 0; point_index < V_target.size(); point_index++)
		{
			points3->InsertNextPoint(V_target[point_index].x, V_target[point_index].y, V_target[point_index].z);
			unsigned char color3[3] = { 100,100,255 };
			colors3->InsertNextTupleValue(color3);
		}

		vtkSmartPointer<vtkPolyData> pointsPolydata3 =
			vtkSmartPointer<vtkPolyData>::New();

		pointsPolydata3->SetPoints(points3);

		vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter3 =
			vtkSmartPointer<vtkVertexGlyphFilter>::New();
		vertexFilter3->SetInputData(pointsPolydata3);
		vertexFilter3->Update();

		vtkSmartPointer<vtkPolyData> polydata3 =
			vtkSmartPointer<vtkPolyData>::New();
		polydata3->ShallowCopy(vertexFilter3->GetOutput());

		polydata3->GetPointData()->SetScalars(colors3);

		vtkSmartPointer<vtkPolyDataMapper> mapper3 =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper3->SetInputData(polydata3);

		vtkSmartPointer<vtkActor> actor3 =
			vtkSmartPointer<vtkActor>::New();
		actor3->SetMapper(mapper3);
		actor3->GetProperty()->SetPointSize(10);

		renderer->AddActor(actor3);
	}

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(20, 20, 20);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	renderer->AddActor(axes);

	if (mm.division_plane.size()>0)
	{
		for (int plane_index = 0; plane_index < mm.division_plane.size(); plane_index++)
		{
			vtkSmartPointer<vtkNamedColors> colors3 =
				vtkSmartPointer<vtkNamedColors>::New();

			//// Set the background color.
			//std::array<unsigned char, 4> bkg{ { 26, 51, 77, 255 } };
			//colors3->SetColor("BkgColor", bkg.data());


			// Create a plane
			vtkSmartPointer<vtkPlaneSource> planeSource =
				vtkSmartPointer<vtkPlaneSource>::New();
			planeSource->SetCenter(mm.division_plane[plane_index].Center.x, mm.division_plane[plane_index].Center.y, mm.division_plane[plane_index].Center.z);
			planeSource->SetNormal(mm.division_plane[plane_index].A, mm.division_plane[plane_index].B, mm.division_plane[plane_index].C);
			planeSource->SetResolution(20, 20);

			planeSource->Update();

			vtkPolyData* plane = planeSource->GetOutput();

			// Create a mapper and actor
			vtkSmartPointer<vtkPolyDataMapper> mapper3 =
				vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper3->SetInputData(plane);

			vtkSmartPointer<vtkActor> actor3 =
				vtkSmartPointer<vtkActor>::New();
			actor3->SetMapper(mapper3);
			actor3->GetProperty()->SetColor(colors3->GetColor3d("White").GetData());

			renderer->AddActor(actor3);
		}

	}

	renderer->SetBackground(0.4392, 0.5020, 0.5647);
	renderer->ResetCamera();

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindow->Render();

	renderWindowInteractor->Start();
}

void MeshRender(vector<float> &Vertices, vector<int32_t> &vertexIndicies)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
	for (size_t face_num = 0; face_num < vertexIndicies.size() / 3; face_num++)
	{
		points->InsertNextPoint(Vertices[vertexIndicies[face_num * 3] * 3], Vertices[vertexIndicies[face_num * 3] * 3 + 1], Vertices[vertexIndicies[face_num * 3] * 3 + 2]);
		points->InsertNextPoint(Vertices[vertexIndicies[face_num * 3 + 1] * 3], Vertices[vertexIndicies[face_num * 3 + 1] * 3 + 1], Vertices[vertexIndicies[face_num * 3 + 1] * 3 + 2]);
		points->InsertNextPoint(Vertices[vertexIndicies[face_num * 3 + 2] * 3], Vertices[vertexIndicies[face_num * 3 + 2] * 3 + 1], Vertices[vertexIndicies[face_num * 3 + 2] * 3 + 2]);
		vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
		polygon->GetPointIds()->SetNumberOfIds(3); //make a triple
		polygon->GetPointIds()->SetId(0, face_num * 3);
		polygon->GetPointIds()->SetId(1, face_num * 3 + 1);
		polygon->GetPointIds()->SetId(2, face_num * 3 + 2);
		polygons->InsertNextCell(polygon);
	}



	vtkSmartPointer<vtkPolyData> triple = vtkSmartPointer<vtkPolyData>::New();
	triple->SetPoints(points);
	cout << points->GetReferenceCount() << endl;
	triple->SetPolys(polygons);

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInput(triple);
#else
	mapper->SetInputData(triple);
#endif

	vtkSmartPointer<vtkActor> texturedQuad =
		vtkSmartPointer<vtkActor>::New();
	texturedQuad->SetMapper(mapper);

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(200, 200, 200);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	/*----------------------------- camera 1 --------------------------*/
	vtkSmartPointer<vtkCamera> camera1 =
		vtkSmartPointer<vtkCamera>::New();
	camera1->SetPosition(100, 50, -100);
	camera1->SetFocalPoint(50, 50, -10);


	// Visualize the textured plane
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(texturedQuad);
	renderer->SetBackground(0.4392, 0.5020, 0.5647);
	renderer->SetActiveCamera(camera1);
	renderer->AddActor(axes);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindow->Render();

	renderWindowInteractor->Start();
}

void MeshRender(vector<float> &Vertices, vector<int32_t> &vertexIndicies, vector<float> &faceTexcoords, cv::Mat &image)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
	for (size_t face_num = 0; face_num < vertexIndicies.size() / 3; face_num++)
	{
		points->InsertNextPoint(Vertices[vertexIndicies[face_num * 3] * 3], Vertices[vertexIndicies[face_num * 3] * 3 + 1], Vertices[vertexIndicies[face_num * 3] * 3 + 2]);
		points->InsertNextPoint(Vertices[vertexIndicies[face_num * 3 + 1] * 3], Vertices[vertexIndicies[face_num * 3 + 1] * 3 + 1], Vertices[vertexIndicies[face_num * 3 + 1] * 3 + 2]);
		points->InsertNextPoint(Vertices[vertexIndicies[face_num * 3 + 2] * 3], Vertices[vertexIndicies[face_num * 3 + 2] * 3 + 1], Vertices[vertexIndicies[face_num * 3 + 2] * 3 + 2]);
		vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
		polygon->GetPointIds()->SetNumberOfIds(3); //make a triple
		polygon->GetPointIds()->SetId(0, face_num * 3);
		polygon->GetPointIds()->SetId(1, face_num * 3 + 1);
		polygon->GetPointIds()->SetId(2, face_num * 3 + 2);
		polygons->InsertNextCell(polygon);
	}

	vtkSmartPointer<vtkFloatArray> textureCoordinates = vtkSmartPointer<vtkFloatArray>::New();
	textureCoordinates->SetNumberOfComponents(2);
	textureCoordinates->SetName("TextureCoordinates");
	float tuple[2];
	for (size_t coor_num = 0; coor_num < faceTexcoords.size() / 6; coor_num++)
	{
		tuple[0] = faceTexcoords[coor_num * 6]; tuple[1] = faceTexcoords[coor_num * 6 + 1];
		textureCoordinates->InsertNextTuple(tuple);
		//outfile << "coor_num " << coor_num << ": "<< tuple[0] * 2048 << "; " << tuple[1] * 2048 << endl;

		tuple[0] = faceTexcoords[coor_num * 6 + 2]; tuple[1] = faceTexcoords[coor_num * 6 + 3];
		textureCoordinates->InsertNextTuple(tuple);
		//outfile << "coor_num " << coor_num << ": " << tuple[0] * 2048 << "; " << tuple[1] * 2048 << endl;

		tuple[0] = faceTexcoords[coor_num * 6 + 4]; tuple[1] = faceTexcoords[coor_num * 6 + 5];
		textureCoordinates->InsertNextTuple(tuple);
		//outfile << "coor_num " << coor_num << ": " << tuple[0] * 2048 << "; " << tuple[1] * 2048 << endl;
	}
	//outfile.close();

	vtkSmartPointer<vtkPolyData> triple = vtkSmartPointer<vtkPolyData>::New();
	triple->SetPoints(points);
	cout << points->GetReferenceCount() << endl;
	triple->SetPolys(polygons);
	triple->GetPointData()->SetTCoords(textureCoordinates);

	cvtColor(image, image, CV_BGR2RGB);
	int width = image.cols;
	int	height = image.rows;
	// Convert the c-style image to a vtkImageData
	vtkSmartPointer<vtkImageImport> imageImport =
		vtkSmartPointer<vtkImageImport>::New();
	imageImport->SetDataSpacing(1, 1, 0);
	imageImport->SetDataOrigin(0, 0, 0);
	imageImport->SetWholeExtent(0, width - 1, 0, height - 1, 0, 0);
	imageImport->SetDataExtentToWholeExtent();
	imageImport->SetDataScalarTypeToUnsignedChar();
	imageImport->SetNumberOfScalarComponents(3);
	//imageImport->SetImportVoidPointer(cImage);
	imageImport->CopyImportVoidPointer(image.data, width * height * 3);
	imageImport->Update();
	vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
	imageData->DeepCopy(imageImport->GetOutput());

	vtkSmartPointer<vtkTexture> texture =
		vtkSmartPointer<vtkTexture>::New();
	texture->SetInputData(imageData);

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInput(triple);
#else
	mapper->SetInputData(triple);
#endif

	vtkSmartPointer<vtkActor> texturedQuad =
		vtkSmartPointer<vtkActor>::New();
	texturedQuad->SetMapper(mapper);
	texturedQuad->SetTexture(texture);

	// Visualize the textured plane
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(texturedQuad);
	renderer->SetBackground(0.4392, 0.5020, 0.5647);
	renderer->ResetCamera();

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindow->Render();

	renderWindowInteractor->Start();
}

void MeshRender(vector<float> &Vertices, vector<int32_t> &vertexIndicies, std::vector<uint8_t> &Colors, vector<float> &faceTexcoords, cv::Mat &image)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	// Setup colors
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");
	unsigned char color[3] = { 255,0,0 };
	vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
	for (size_t face_num = 0; face_num < vertexIndicies.size() / 3; face_num++)
	{
		points->InsertNextPoint(Vertices[vertexIndicies[face_num * 3] * 3], Vertices[vertexIndicies[face_num * 3] * 3 + 1], Vertices[vertexIndicies[face_num * 3] * 3 + 2]);
		points->InsertNextPoint(Vertices[vertexIndicies[face_num * 3 + 1] * 3], Vertices[vertexIndicies[face_num * 3 + 1] * 3 + 1], Vertices[vertexIndicies[face_num * 3 + 1] * 3 + 2]);
		points->InsertNextPoint(Vertices[vertexIndicies[face_num * 3 + 2] * 3], Vertices[vertexIndicies[face_num * 3 + 2] * 3 + 1], Vertices[vertexIndicies[face_num * 3 + 2] * 3 + 2]);
		vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
		polygon->GetPointIds()->SetNumberOfIds(3); //make a triple
		polygon->GetPointIds()->SetId(0, face_num * 3);
		polygon->GetPointIds()->SetId(1, face_num * 3 + 1);
		polygon->GetPointIds()->SetId(2, face_num * 3 + 2);
		polygons->InsertNextCell(polygon);


		colors->InsertNextTupleValue(color);
		colors->InsertNextTupleValue(color);
		colors->InsertNextTupleValue(color);
	}

	vtkSmartPointer<vtkFloatArray> textureCoordinates = vtkSmartPointer<vtkFloatArray>::New();
	textureCoordinates->SetNumberOfComponents(2);
	textureCoordinates->SetName("TextureCoordinates");
	float tuple[2];
	for (size_t coor_num = 0; coor_num < faceTexcoords.size() / 6; coor_num++)
	{
		tuple[0] = faceTexcoords[coor_num * 6]; tuple[1] = faceTexcoords[coor_num * 6 + 1];
		textureCoordinates->InsertNextTuple(tuple);
		//outfile << "coor_num " << coor_num << ": "<< tuple[0] * 2048 << "; " << tuple[1] * 2048 << endl;

		tuple[0] = faceTexcoords[coor_num * 6 + 2]; tuple[1] = faceTexcoords[coor_num * 6 + 3];
		textureCoordinates->InsertNextTuple(tuple);
		//outfile << "coor_num " << coor_num << ": " << tuple[0] * 2048 << "; " << tuple[1] * 2048 << endl;

		tuple[0] = faceTexcoords[coor_num * 6 + 4]; tuple[1] = faceTexcoords[coor_num * 6 + 5];
		textureCoordinates->InsertNextTuple(tuple);
		//outfile << "coor_num " << coor_num << ": " << tuple[0] * 2048 << "; " << tuple[1] * 2048 << endl;
	}
	//outfile.close();

	vtkSmartPointer<vtkPolyData> triple = vtkSmartPointer<vtkPolyData>::New();
	triple->SetPoints(points);
	cout << points->GetReferenceCount() << endl;
	triple->SetPolys(polygons);

	triple->GetPointData()->SetScalars(colors);
	triple->GetPointData()->SetTCoords(textureCoordinates);
	cv::cvtColor(image, image, CV_BGR2RGB);
	int width = image.cols;
	int	height = image.rows;
	// Convert the c-style image to a vtkImageData
	vtkSmartPointer<vtkImageImport> imageImport =
		vtkSmartPointer<vtkImageImport>::New();
	imageImport->SetDataSpacing(1, 1, 0);
	imageImport->SetDataOrigin(0, 0, 0);
	imageImport->SetWholeExtent(0, width - 1, 0, height - 1, 0, 0);
	imageImport->SetDataExtentToWholeExtent();
	imageImport->SetDataScalarTypeToUnsignedChar();
	imageImport->SetNumberOfScalarComponents(3);
	//imageImport->SetImportVoidPointer(cImage);
	imageImport->CopyImportVoidPointer(image.data, width * height * 3);
	imageImport->Update();
	vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
	imageData->DeepCopy(imageImport->GetOutput());

	vtkSmartPointer<vtkTexture> texture =
		vtkSmartPointer<vtkTexture>::New();
	texture->SetInputData(imageData);

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInput(triple);
#else
	mapper->SetInputData(triple);
#endif

	vtkSmartPointer<vtkActor> texturedQuad =
		vtkSmartPointer<vtkActor>::New();
	texturedQuad->SetMapper(mapper);
	texturedQuad->SetTexture(texture);

	// Visualize the textured plane
	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(texturedQuad);
	renderer->SetBackground(0.4392, 0.5020, 0.5647);
	renderer->ResetCamera();

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindow->Render();

	renderWindowInteractor->Start();
}

int PointRender(vector<float> &Vertices)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	//points->SetNumberOfPoints(4);

	for (size_t points_num = 0; points_num < Vertices.size() / 3; points_num++)
	{
		//if (points_num>2)
		//{
		//	continue;
		//}
		points->InsertNextPoint(Vertices[points_num * 3], Vertices[points_num * 3 + 1], Vertices[points_num * 3 + 2]);
		//cout << "U = " << Vertices[points_num * 3] * 100 << "; V = " << Vertices[points_num * 3 + 1] * 100 << "; Z = " << Vertices[points_num * 3 + 2] * 100 << endl;

		//colors->InsertNextTupleValue(red);
	}

	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);

	vtkSmartPointer<vtkNamedColors> colors =
		vtkSmartPointer<vtkNamedColors>::New();
	///
	double radius = 0.001;
	vtkSmartPointer<vtkSphereSource> sphereSource1 =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource1->SetRadius(radius);

	vtkSmartPointer<vtkGlyph3DMapper> glyph3D1 =
		vtkSmartPointer<vtkGlyph3DMapper>::New();
	glyph3D1->SetInputData(polyData);
	glyph3D1->SetSourceConnection(sphereSource1->GetOutputPort());

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(2, 2, 2);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	vtkSmartPointer<vtkActor> glyph3DActor1 =
		vtkSmartPointer<vtkActor>::New();
	glyph3DActor1->SetMapper(glyph3D1);
	glyph3DActor1->GetProperty()->SetColor(colors->GetColor3d("LimeGreen").GetData());

	// Create graphics stuff
	//
	vtkSmartPointer<vtkRenderer> ren1 =
		vtkSmartPointer<vtkRenderer>::New();
	ren1->SetBackground(colors->GetColor3d("White").GetData());
	ren1->AddActor(glyph3DActor1);
	ren1->GetActiveCamera()->SetPosition(5, 1, 1);
	ren1->GetActiveCamera()->SetFocalPoint(1, 1, 1);
	//ren1->GetActiveCamera()->SetViewUp(0, 0, 1);
	ren1->ResetCamera();
	ren1->GetActiveCamera()->Dolly(1.0);
	ren1->ResetCameraClippingRange();
	ren1->AddActor(axes);

	vtkSmartPointer<vtkRenderWindow> renWin =
		vtkSmartPointer<vtkRenderWindow>::New();
	renWin->AddRenderer(ren1);
	renWin->SetSize(512, 512);

	vtkSmartPointer<vtkRenderWindowInteractor> iren =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//


	iren->Initialize();
	iren->Start();

	return EXIT_SUCCESS;
}

int PointRender(vector<float> &Vertices, vector<uint8_t> colors_in)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	//points->SetNumberOfPoints(4);

	for (size_t points_num = 0; points_num < Vertices.size() / 3; points_num++)
	{
		//if (points_num>2)
		//{
		//	continue;
		//}
		points->InsertNextPoint(Vertices[points_num * 3], Vertices[points_num * 3 + 1], Vertices[points_num * 3 + 2]);
		//cout << "U = " << Vertices[points_num * 3] * 100 << "; V = " << Vertices[points_num * 3 + 1] * 100 << "; Z = " << Vertices[points_num * 3 + 2] * 100 << endl;

		//colors->InsertNextTupleValue(red);
	}

	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);

	vtkSmartPointer<vtkNamedColors> colors =
		vtkSmartPointer<vtkNamedColors>::New();
	///
	double radius = 0.001;
	vtkSmartPointer<vtkSphereSource> sphereSource1 =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource1->SetRadius(radius);

	vtkSmartPointer<vtkGlyph3DMapper> glyph3D1 =
		vtkSmartPointer<vtkGlyph3DMapper>::New();
	glyph3D1->SetInputData(polyData);
	glyph3D1->SetSourceConnection(sphereSource1->GetOutputPort());

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(2, 2, 2);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	vtkSmartPointer<vtkActor> glyph3DActor1 =
		vtkSmartPointer<vtkActor>::New();
	glyph3DActor1->SetMapper(glyph3D1);
	glyph3DActor1->GetProperty()->SetColor(colors->GetColor3d("LimeGreen").GetData());

	// Create graphics stuff
	//
	vtkSmartPointer<vtkRenderer> ren1 =
		vtkSmartPointer<vtkRenderer>::New();
	ren1->SetBackground(colors->GetColor3d("White").GetData());
	ren1->AddActor(glyph3DActor1);
	ren1->GetActiveCamera()->SetPosition(5, 1, 1);
	ren1->GetActiveCamera()->SetFocalPoint(1, 1, 1);
	//ren1->GetActiveCamera()->SetViewUp(0, 0, 1);
	ren1->ResetCamera();
	ren1->GetActiveCamera()->Dolly(1.0);
	ren1->ResetCameraClippingRange();
	ren1->AddActor(axes);

	vtkSmartPointer<vtkRenderWindow> renWin =
		vtkSmartPointer<vtkRenderWindow>::New();
	renWin->AddRenderer(ren1);
	renWin->SetSize(512, 512);

	vtkSmartPointer<vtkRenderWindowInteractor> iren =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//


	iren->Initialize();
	iren->Start();

	return EXIT_SUCCESS;
}

int PointRender(vector<float> &Vertices, vector<float> &Vertices2)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();

	for (size_t points_num = 0; points_num < Vertices.size() / 3; points_num++)
	{
		points->InsertNextPoint(Vertices[points_num * 3], Vertices[points_num * 3 + 1], Vertices[points_num * 3 + 2]);
	}

	for (size_t points_num = 0; points_num < Vertices2.size() / 3; points_num++)
	{
		points2->InsertNextPoint(Vertices2[points_num * 3], Vertices2[points_num * 3 + 1], Vertices2[points_num * 3 + 2]);
	}

	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);

	vtkSmartPointer<vtkPolyData> polyData2 = vtkSmartPointer<vtkPolyData>::New();
	polyData2->SetPoints(points2);

	vtkSmartPointer<vtkNamedColors> colors =
		vtkSmartPointer<vtkNamedColors>::New();
	///
	double radius = 0.001;
	vtkSmartPointer<vtkSphereSource> sphereSource1 =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource1->SetRadius(radius);
	vtkSmartPointer<vtkSphereSource> sphereSource2 =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource2->SetRadius(radius);

	vtkSmartPointer<vtkGlyph3DMapper> glyph3D1 =
		vtkSmartPointer<vtkGlyph3DMapper>::New();
	glyph3D1->SetInputData(polyData);
	glyph3D1->SetSourceConnection(sphereSource1->GetOutputPort());

	vtkSmartPointer<vtkGlyph3DMapper> glyph3D2 =
		vtkSmartPointer<vtkGlyph3DMapper>::New();
	glyph3D2->SetInputData(polyData2);
	glyph3D2->SetSourceConnection(sphereSource2->GetOutputPort());

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();

	vtkSmartPointer<vtkActor> glyph3DActor1 =
		vtkSmartPointer<vtkActor>::New();
	glyph3DActor1->SetMapper(glyph3D1);
	glyph3DActor1->GetProperty()->SetColor(colors->GetColor3d("LimeGreen").GetData());

	vtkSmartPointer<vtkActor> glyph3DActor2 =
		vtkSmartPointer<vtkActor>::New();
	glyph3DActor2->SetMapper(glyph3D2);
	glyph3DActor2->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());

	// Create graphics stuff
	//
	vtkSmartPointer<vtkRenderer> ren1 =
		vtkSmartPointer<vtkRenderer>::New();
	ren1->SetBackground(colors->GetColor3d("White").GetData());
	ren1->AddActor(glyph3DActor1);
	ren1->AddActor(glyph3DActor2);
	ren1->GetActiveCamera()->SetPosition(1, 0, 0);
	ren1->GetActiveCamera()->SetFocalPoint(0, 1, 0);
	ren1->GetActiveCamera()->SetViewUp(0, 0, 1);
	ren1->ResetCamera();
	ren1->GetActiveCamera()->Dolly(1.0);
	ren1->ResetCameraClippingRange();
	ren1->AddActor(axes);

	vtkSmartPointer<vtkRenderWindow> renWin =
		vtkSmartPointer<vtkRenderWindow>::New();
	renWin->AddRenderer(ren1);
	renWin->SetSize(512, 512);

	vtkSmartPointer<vtkRenderWindowInteractor> iren =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//


	iren->Initialize();
	iren->Start();

	return EXIT_SUCCESS;
}

int PointRender2(vector<double> &Vertices, vector<double> &Vertices2)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();

	for (size_t points_num = 0; points_num < Vertices.size() / 3; points_num++)
	{
		points->InsertNextPoint(Vertices[points_num * 3], Vertices[points_num * 3 + 1], Vertices[points_num * 3 + 2]);
	}

	for (size_t points_num = 0; points_num < Vertices2.size() / 3; points_num++)
	{
		points2->InsertNextPoint(Vertices2[points_num * 3], Vertices2[points_num * 3 + 1], Vertices2[points_num * 3 + 2]);
	}

	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);

	vtkSmartPointer<vtkPolyData> polyData2 = vtkSmartPointer<vtkPolyData>::New();
	polyData2->SetPoints(points2);

	vtkSmartPointer<vtkNamedColors> colors =
		vtkSmartPointer<vtkNamedColors>::New();
	///
	double radius = 0.1;
	vtkSmartPointer<vtkSphereSource> sphereSource1 =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource1->SetRadius(radius);
	vtkSmartPointer<vtkSphereSource> sphereSource2 =
		vtkSmartPointer<vtkSphereSource>::New();
	sphereSource2->SetRadius(radius);

	vtkSmartPointer<vtkGlyph3DMapper> glyph3D1 =
		vtkSmartPointer<vtkGlyph3DMapper>::New();
	glyph3D1->SetInputData(polyData);
	glyph3D1->SetSourceConnection(sphereSource1->GetOutputPort());

	vtkSmartPointer<vtkGlyph3DMapper> glyph3D2 =
		vtkSmartPointer<vtkGlyph3DMapper>::New();
	glyph3D2->SetInputData(polyData2);
	glyph3D2->SetSourceConnection(sphereSource2->GetOutputPort());

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(20, 20, 20);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	vtkSmartPointer<vtkActor> glyph3DActor1 =
		vtkSmartPointer<vtkActor>::New();
	glyph3DActor1->SetMapper(glyph3D1);
	glyph3DActor1->GetProperty()->SetColor(colors->GetColor3d("LimeGreen").GetData());

	vtkSmartPointer<vtkActor> glyph3DActor2 =
		vtkSmartPointer<vtkActor>::New();
	glyph3DActor2->SetMapper(glyph3D2);
	glyph3DActor2->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());

	// Create graphics stuff
	//
	vtkSmartPointer<vtkRenderer> ren1 =
		vtkSmartPointer<vtkRenderer>::New();
	ren1->SetBackground(colors->GetColor3d("White").GetData());
	ren1->AddActor(glyph3DActor1);
	ren1->AddActor(glyph3DActor2);

	vtkSmartPointer<vtkCamera> camera1 =
		vtkSmartPointer<vtkCamera>::New();
	double camera_view[3] = { -3, 0, -3 };
	double camera_focal[3] = { 0, 0, 0 };
	camera1->SetPosition(camera_view);
	camera1->SetFocalPoint(camera_focal);
	ren1->SetActiveCamera(camera1);
	ren1->AddActor(axes);

	vtkSmartPointer<vtkRenderWindow> renWin =
		vtkSmartPointer<vtkRenderWindow>::New();
	renWin->AddRenderer(ren1);
	renWin->SetSize(1024, 1024);

	vtkSmartPointer<vtkRenderWindowInteractor> iren =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iren->SetRenderWindow(renWin);

	// Add the actors to the renderer, set the background and size
	//


	iren->Initialize();
	iren->Start();

	return EXIT_SUCCESS;
}

int ColoredPoints(vector<double> &Vertices, double size_)
{
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
		vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	double min_dis = Vertices[1];
	double max_dis = Vertices[1];
	for (int point_index = 1; point_index < Vertices.size() / 3; point_index++)
	{
		if (Vertices[3 * point_index + 1]>max_dis)
		{
			max_dis = Vertices[3 * point_index + 1];
		}
		if (Vertices[3 * point_index + 1]<min_dis)
		{
			min_dis = Vertices[3 * point_index + 1];
		}
	}
	double l = max_dis - min_dis;
	for (int point_index = 0; point_index < Vertices.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices[3 * point_index + 0], Vertices[3 * point_index + 1], Vertices[3 * point_index + 2]);
		unsigned char color[3] = { (Vertices[3 * point_index + 1] - min_dis)*255.0 / l,255 - (Vertices[3 * point_index + 1] - min_dis)*255.0 / l,127 };
		colors->InsertNextTupleValue(color);
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());

	polydata->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(20, 20, 20);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	// Visualization

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(size_);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();

	/*----------------------------- camera 1 --------------------------*/
	//vtkSmartPointer<vtkCamera> camera1 =
	//	vtkSmartPointer<vtkCamera>::New();
	//camera1->SetPosition(300,50,-300);
	//camera1->SetFocalPoint(50,50,-10);
	//renderer->SetActiveCamera(camera1);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkAreaPicker> areaPicker =
		vtkSmartPointer<vtkAreaPicker>::New();
	renderWindowInteractor->SetPicker(areaPicker);

	renderer->AddActor(axes);
	renderer->AddActor(actor);
	renderer->SetBackground(namedColors->GetColor3d("White").GetData());
	renderWindow->Render();

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

int ColoredPoints2(vector<double> &Vertices, vector<double> &Vertices2, double size_)
{
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
		vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	//double min_dis = Vertices[1];
	//double max_dis = Vertices[1];
	//for (int point_index = 1; point_index < Vertices.size() / 3; point_index++)
	//{
	//	if (Vertices[3 * point_index + 1]>max_dis)
	//	{
	//		max_dis = Vertices[3 * point_index + 1];
	//	}
	//	if (Vertices[3 * point_index + 1]<min_dis)
	//	{
	//		min_dis = Vertices[3 * point_index + 1];
	//	}
	//}
	//double l = max_dis - min_dis;
	for (int point_index = 0; point_index < Vertices.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices[3 * point_index + 0], Vertices[3 * point_index + 1], Vertices[3 * point_index + 2]);
		unsigned char color[3] = { 10,255,127 };
		colors->InsertNextTupleValue(color);
	}
	for (int point_index = 0; point_index < Vertices2.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices2[3 * point_index + 0], Vertices2[3 * point_index + 1], Vertices2[3 * point_index + 2]);
		unsigned char color[3] = { 255.0 ,10,127 };
		colors->InsertNextTupleValue(color);
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());

	polydata->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(20, 20, 20);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	// Visualization

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(size_);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();

	/*----------------------------- camera 1 --------------------------*/
	vtkSmartPointer<vtkCamera> camera1 =
		vtkSmartPointer<vtkCamera>::New();
	camera1->SetPosition(300, 50, -300);
	camera1->SetFocalPoint(50, 50, -10);
	renderer->SetActiveCamera(camera1);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkAreaPicker> areaPicker =
		vtkSmartPointer<vtkAreaPicker>::New();
	renderWindowInteractor->SetPicker(areaPicker);

	renderer->AddActor(axes);
	renderer->AddActor(actor);
	renderer->SetBackground(namedColors->GetColor3d("White").GetData());
	renderWindow->Render();

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

int ColoredPoints2(vector<double> &Vertices, vector<double> &Vertices2, double size_, vector<orth::Plane> &planes)
{
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
		vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	//double min_dis = Vertices[1];
	//double max_dis = Vertices[1];
	//for (int point_index = 1; point_index < Vertices.size() / 3; point_index++)
	//{
	//	if (Vertices[3 * point_index + 1]>max_dis)
	//	{
	//		max_dis = Vertices[3 * point_index + 1];
	//	}
	//	if (Vertices[3 * point_index + 1]<min_dis)
	//	{
	//		min_dis = Vertices[3 * point_index + 1];
	//	}
	//}
	//double l = max_dis - min_dis;
	for (int point_index = 0; point_index < Vertices.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices[3 * point_index + 0], Vertices[3 * point_index + 1], Vertices[3 * point_index + 2]);
		unsigned char color[3] = { 10,255,127 };
		colors->InsertNextTupleValue(color);
	}
	for (int point_index = 0; point_index < Vertices2.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices2[3 * point_index + 0], Vertices2[3 * point_index + 1], Vertices2[3 * point_index + 2]);
		unsigned char color[3] = { 255.0 ,10,127 };
		colors->InsertNextTupleValue(color);
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());

	polydata->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(20, 20, 20);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	// Visualization

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(size_);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();

	/*----------------------------- camera 1 --------------------------*/
	vtkSmartPointer<vtkCamera> camera1 =
		vtkSmartPointer<vtkCamera>::New();
	camera1->SetPosition(300, 50, -300);
	camera1->SetFocalPoint(50, 50, -10);
	renderer->SetActiveCamera(camera1);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkAreaPicker> areaPicker =
		vtkSmartPointer<vtkAreaPicker>::New();
	renderWindowInteractor->SetPicker(areaPicker);

	if (planes.size()>0)
	{
		for (int plane_index = 0; plane_index < planes.size(); plane_index++)
		{
			vtkSmartPointer<vtkNamedColors> colors3 =
				vtkSmartPointer<vtkNamedColors>::New();

			//// Set the background color.
			//std::array<unsigned char, 4> bkg{ { 26, 51, 77, 255 } };
			//colors3->SetColor("BkgColor", bkg.data());

			// Create a plane
			vtkSmartPointer<vtkPlaneSource> planeSource =
				vtkSmartPointer<vtkPlaneSource>::New();
			planeSource->SetCenter(planes[plane_index].Center.x, planes[plane_index].Center.y, planes[plane_index].Center.z);
			planeSource->SetNormal(planes[plane_index].A, planes[plane_index].B, planes[plane_index].C);
			planeSource->SetResolution(20, 20);

			planeSource->Update();

			vtkPolyData* plane = planeSource->GetOutput();

			// Create a mapper and actor
			vtkSmartPointer<vtkPolyDataMapper> mapper3 =
				vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper3->SetInputData(plane);

			vtkSmartPointer<vtkActor> actor3 =
				vtkSmartPointer<vtkActor>::New();
			actor3->SetMapper(mapper3);
			actor3->GetProperty()->SetColor(colors3->GetColor3d("Yellow").GetData());

			renderer->AddActor(actor3);
		}

	}

	renderer->AddActor(axes);
	renderer->AddActor(actor);
	renderer->SetBackground(namedColors->GetColor3d("White").GetData());
	renderWindow->Render();

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

int ColoredPoints3(vector<double> &Vertices, vector<double> &Vertices2, vector<double> &Vertices3, double size_)
{
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
		vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	//double min_dis = Vertices[1];
	//double max_dis = Vertices[1];
	//for (int point_index = 1; point_index < Vertices.size() / 3; point_index++)
	//{
	//	if (Vertices[3 * point_index + 1]>max_dis)
	//	{
	//		max_dis = Vertices[3 * point_index + 1];
	//	}
	//	if (Vertices[3 * point_index + 1]<min_dis)
	//	{
	//		min_dis = Vertices[3 * point_index + 1];
	//	}
	//}
	//double l = max_dis - min_dis;
	for (int point_index = 0; point_index < Vertices.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices[3 * point_index + 0], Vertices[3 * point_index + 1], Vertices[3 * point_index + 2]);
		unsigned char color[3] = { 10,255,127 };
		colors->InsertNextTupleValue(color);
	}
	for (int point_index = 0; point_index < Vertices2.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices2[3 * point_index + 0], Vertices2[3 * point_index + 1], Vertices2[3 * point_index + 2]);
		unsigned char color[3] = { 255.0 ,10,127 };
		colors->InsertNextTupleValue(color);
	}
	for (int point_index = 0; point_index < Vertices3.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices3[3 * point_index + 0], Vertices3[3 * point_index + 1], Vertices3[3 * point_index + 2]);
		unsigned char color[3] = { 127.0 ,10,255 };
		colors->InsertNextTupleValue(color);
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());

	polydata->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(20, 20, 20);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	// Visualization

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(size_);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();

	/*----------------------------- camera 1 --------------------------*/
	vtkSmartPointer<vtkCamera> camera1 =
		vtkSmartPointer<vtkCamera>::New();
	camera1->SetPosition(300, 50, -300);
	camera1->SetFocalPoint(50, 50, -10);
	renderer->SetActiveCamera(camera1);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkAreaPicker> areaPicker =
		vtkSmartPointer<vtkAreaPicker>::New();
	renderWindowInteractor->SetPicker(areaPicker);

	renderer->AddActor(axes);
	renderer->AddActor(actor);
	renderer->SetBackground(namedColors->GetColor3d("White").GetData());
	renderWindow->Render();

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

int ColoredPoints(vector<float> &Vertices, std::vector<uint8_t> &Colors)
{
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
		vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	for (int point_index = 0; point_index < Vertices.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices[3 * point_index + 0], Vertices[3 * point_index + 1], Vertices[3 * point_index + 2]);
		unsigned char color[3] = { Colors[3 * point_index + 0],Colors[3 * point_index + 1],Colors[3 * point_index + 2] };
		colors->InsertNextTupleValue(color);
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());



	polydata->GetPointData()->SetScalars(colors);

	// Visualization
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(3);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderer->AddActor(actor);
	renderer->SetBackground(namedColors->GetColor3d("Burlywood").GetData());
	renderWindow->Render();
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

int ColoredPoints(vector<double> &Vertices, std::vector<uint8_t> &Colors)
{
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
		vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	for (int point_index = 0; point_index < Vertices.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices[3 * point_index + 0], Vertices[3 * point_index + 1], Vertices[3 * point_index + 2]);
		unsigned char color[3] = { Colors[3 * point_index + 0],Colors[3 * point_index + 1],Colors[3 * point_index + 2] };
		colors->InsertNextTupleValue(color);
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());



	polydata->GetPointData()->SetScalars(colors);

	// Visualization
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(2);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderer->AddActor(actor);
	renderer->SetBackground(namedColors->GetColor3d("Burlywood").GetData());
	renderWindow->Render();
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

void SelectPoints(vector<double> &Vertices)
{
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	for (int point_index = 0; point_index < Vertices.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices[3 * point_index + 0], Vertices[3 * point_index + 1], Vertices[3 * point_index + 2]);
	}
	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkIdFilter> idFilter =
		vtkSmartPointer<vtkIdFilter>::New();
	idFilter->SetInputConnection(vertexFilter->GetOutputPort());
	//idFilter->SetInputData(points);
	idFilter->SetIdsArrayName("OriginalIds");
	idFilter->Update();

	vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter =
		vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
	surfaceFilter->SetInputConnection(idFilter->GetOutputPort());
	surfaceFilter->Update();

	vtkPolyData* input = surfaceFilter->GetOutput();

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
	mapper->SetInputConnection(input->GetProducerPort());
#else
	mapper->SetInputData(input);
#endif
	mapper->ScalarVisibilityOff();

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	// Visualize
	//vtkSmartPointer<vtkAxesActor> axes =
	//	vtkSmartPointer<vtkAxesActor>::New();
	//axes->SetShaftTypeToLine();
	//axes->SetTotalLength(2, 2, 2);
	//axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	//axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);

	vtkSmartPointer<vtkAreaPicker> areaPicker =
		vtkSmartPointer<vtkAreaPicker>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetPicker(areaPicker);
	renderWindowInteractor->SetRenderWindow(renderWindow);
	//renderer->AddActor(axes);
	renderer->AddActor(actor);
	//renderer->SetBackground(1,1,1); // Background color white

	renderWindow->Render();

	vtkSmartPointer<InteractorStyle> style =
		vtkSmartPointer<InteractorStyle>::New();
	style->SetPoints(input);
	renderWindowInteractor->SetInteractorStyle(style);

	renderWindowInteractor->Start();
}

void ColoredImage3DDistribution(cv::Mat &image_in, double size_)
{
	vector<float> Vertices;

	for (int v = 0; v < image_in.rows; v++)
	{

		for (int u = 0; u < image_in.cols; u++)
		{
			if (image_in.at<double>(v, u) != 0)
			{
				Vertices.push_back((float)u);
				Vertices.push_back((float)v);
				Vertices.push_back(image_in.at<double>(v, u));
			}

		}
	}


	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
		vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	double min_dis = Vertices[2];
	double max_dis = Vertices[2];
	for (int point_index = 1; point_index < Vertices.size() / 3; point_index++)
	{
		if (Vertices[3 * point_index + 2]>max_dis)
		{
			max_dis = Vertices[3 * point_index + 2];
		}
		if (Vertices[3 * point_index + 2]<min_dis)
		{
			min_dis = Vertices[3 * point_index + 2];
		}
	}
	double l = max_dis - min_dis;
	for (int point_index = 0; point_index < Vertices.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices[3 * point_index + 0], Vertices[3 * point_index + 1], Vertices[3 * point_index + 2]);
		unsigned char color[3] = { (Vertices[3 * point_index + 2] - min_dis)*255.0 / l,255 - (Vertices[3 * point_index + 2] - min_dis)*255.0 / l,127 };
		colors->InsertNextTupleValue(color);
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());

	polydata->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(100, 100, 100);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	// Visualization

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(size_);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();

	/*----------------------------- camera 1 --------------------------*/
	vtkSmartPointer<vtkCamera> camera1 =
		vtkSmartPointer<vtkCamera>::New();
	camera1->SetPosition(300, 50, -300);
	camera1->SetFocalPoint(50, 50, -10);
	renderer->SetActiveCamera(camera1);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkAreaPicker> areaPicker =
		vtkSmartPointer<vtkAreaPicker>::New();
	renderWindowInteractor->SetPicker(areaPicker);

	renderer->AddActor(axes);
	renderer->AddActor(actor);
	renderer->SetBackground(namedColors->GetColor3d("White").GetData());
	renderWindow->Render();

	renderWindowInteractor->Start();
}

void Colored2Image3DDistribution(cv::Mat &image_in, cv::Mat &image_in2, double size_)
{
	vector<float> Vertices;

	for (int v = 0; v < image_in.rows; v++)
	{

		for (int u = 0; u < image_in.cols; u++)
		{

			Vertices.push_back((float)u);
			Vertices.push_back((float)v);
			Vertices.push_back(image_in.at<float>(v, u) * 10);
		}
	}

	for (int v = 0; v < image_in2.rows; v++)
	{

		for (int u = 0; u < image_in2.cols; u++)
		{

			Vertices.push_back((float)u);
			Vertices.push_back((float)v);
			Vertices.push_back(image_in2.at<float>(v, u) * 10);
		}
	}


	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
		vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	double min_dis = Vertices[2];
	double max_dis = Vertices[2];
	for (int point_index = 1; point_index < Vertices.size() / 3; point_index++)
	{
		if (Vertices[3 * point_index + 2]>max_dis)
		{
			max_dis = Vertices[3 * point_index + 2];
		}
		if (Vertices[3 * point_index + 2]<min_dis)
		{
			min_dis = Vertices[3 * point_index + 2];
		}
	}
	double l = max_dis - min_dis;
	for (int point_index = 0; point_index < Vertices.size() / 3; point_index++)
	{
		points->InsertNextPoint(Vertices[3 * point_index + 0], Vertices[3 * point_index + 1], Vertices[3 * point_index + 2]);
		unsigned char color[3] = { (Vertices[3 * point_index + 2] - min_dis)*255.0 / l,255 - (Vertices[3 * point_index + 2] - min_dis)*255.0 / l,127 };
		colors->InsertNextTupleValue(color);
	}

	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());

	polydata->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(100, 100, 100);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	// Visualization

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(size_);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();

	/*----------------------------- camera 1 --------------------------*/
	vtkSmartPointer<vtkCamera> camera1 =
		vtkSmartPointer<vtkCamera>::New();
	camera1->SetPosition(300, 50, -300);
	camera1->SetFocalPoint(50, 50, -10);
	renderer->SetActiveCamera(camera1);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkAreaPicker> areaPicker =
		vtkSmartPointer<vtkAreaPicker>::New();
	renderWindowInteractor->SetPicker(areaPicker);

	renderer->AddActor(axes);
	renderer->AddActor(actor);
	renderer->SetBackground(namedColors->GetColor3d("White").GetData());
	renderWindow->Render();

	renderWindowInteractor->Start();
}

int ColoredTSDF(vector<float> &Vertices, int resolution_x, int resolution_y, int resolution_z, double size_)
{
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();

	// Setup colors
	vtkSmartPointer<vtkNamedColors> namedColors =
		vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");



	//double min_dis = Vertices[1];
	//double max_dis = Vertices[1];
	//for (int point_index = 1; point_index < Vertices.size() / 3; point_index++)
	//{
	//	if (Vertices[3 * point_index + 1]>max_dis)
	//	{
	//		max_dis = Vertices[3 * point_index + 1];
	//	}
	//	if (Vertices[3 * point_index + 1]<min_dis)
	//	{
	//		min_dis = Vertices[3 * point_index + 1];
	//	}
	//}
	//double l = max_dis - min_dis;

	for (int point_index = 0; point_index < Vertices.size(); point_index++)
	{
		int x = point_index / (resolution_y * resolution_z);
		int y = (point_index % (resolution_y * resolution_z)) / resolution_z;
		int z = (point_index % (resolution_y * resolution_z)) % resolution_z;
		double cc = Vertices[x*resolution_y*resolution_z + y*resolution_z + z];

		if ((cc != 0) && (cc != -20) && (abs(cc) < 0.3))
		{
			points->InsertNextPoint(x, y, z);
			unsigned char r = (-255 * cc) > 0 ? (-255 * cc) : 0;
			unsigned char g = 255 * (1 - 2 * abs(cc)) > 0 ? 255 * (1 - 2 * abs(cc)) : 0;
			unsigned char b = (255 * cc) > 0 ? (255 * cc) : 0;
			unsigned char color[3] = { r , g, b };
			colors->InsertNextTupleValue(color);
		}

	}

	vtkSmartPointer<vtkPolyData> pointsPolydata =
		vtkSmartPointer<vtkPolyData>::New();

	pointsPolydata->SetPoints(points);

	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
		vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());

	polydata->GetPointData()->SetScalars(colors);

	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();
	axes->SetShaftTypeToLine();
	axes->SetTotalLength(resolution_x, resolution_y, resolution_z);
	axes->SetNormalizedShaftLength(1.0, 1.0, 1.0);
	axes->SetNormalizedTipLength(0.05, 0.05, 0.05);

	// Visualization

	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(size_);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();

	/*----------------------------- camera 1 --------------------------*/
	vtkSmartPointer<vtkCamera> camera1 =
		vtkSmartPointer<vtkCamera>::New();
	camera1->SetPosition(300, 50, -300);
	camera1->SetFocalPoint(50, 50, -10);
	renderer->SetActiveCamera(camera1);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	renderWindow->SetSize(1000, 1000);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renderWindowInteractor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkAreaPicker> areaPicker =
		vtkSmartPointer<vtkAreaPicker>::New();
	renderWindowInteractor->SetPicker(areaPicker);

	renderer->AddActor(axes);
	renderer->AddActor(actor);
	renderer->SetBackground(namedColors->GetColor3d("White").GetData());
	renderWindow->Render();

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}