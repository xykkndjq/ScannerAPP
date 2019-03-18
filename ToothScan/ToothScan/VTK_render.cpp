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

			Vertices.push_back((float)u);
			Vertices.push_back((float)v);
			Vertices.push_back(image_in.at<double>(v, u) * 10);
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

		if (cc != 0)
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