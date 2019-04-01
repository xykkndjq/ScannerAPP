#pragma once

#include "vtkAutoInit.h" 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <iostream>
#include <opencv2\opencv.hpp>
#include <vtkCallbackCommand.h>
#include <vtkVersion.h>
#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkDoubleArray.h>
#include <vtkLine.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTexture.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkFloatArray.h>
#include <vtkPolygon.h>
#include <vtkImageImport.h>
#include <vtkDensifyPointCloudFilter.h>
#include <vtkSphereSource.h>
#include <vtkLineSource.h>
#include <vtkGlyph3DMapper.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkNamedColors.h>
#include <vtksys/SystemTools.hxx>
#include <vtkAxesActor.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkDataSetMapper.h>
#include <vtkUnstructuredGrid.h>
#include <vtkIdFilter.h>
#include <vtkIdTypeArray.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkPlanes.h>
#include <vtkPlaneSource.h>
#include <vtkPointSource.h>
#include <vtkProgrammableFilter.h>

#include "basetype.h"


// For compatibility with new VTK generic data arrays
#ifdef vtkGenericDataArray_h
#define InsertNextTupleValue InsertNextTypedTuple
#endif

using std::vector;
using std::cout;
using std::endl;
using std::string;

void MeshRender(orth::MeshModel &mm, float size_);
void MeshRender(orth::MaxillaryTeeth &mm);
void MeshRender(orth::MaxillaryTeeth &mm, orth::PointCloudD &V_query, orth::PointCloudD &V_target);

void MeshRender(vector<float> &Vertices, vector<int32_t> &vertexIndicies);
void MeshRender(vector<float> &Vertices, vector<int32_t> &vertexIndicies, vector<float> &faceTexcoords, cv::Mat &image);
void MeshRender(vector<float> &Vertices, vector<int32_t> &vertexIndicies, std::vector<uint8_t> &colors, vector<float> &faceTexcoords, cv::Mat &image);
int PointRender(vector<float> &Vertices);
int PointRender(vector<float> &Vertices, vector<uint8_t> colors);
int PointRender(vector<float> &Vertices, vector<float> &Vertices2);
int PointRender2(vector<double> &Vertices, vector<double> &Vertices2);
int ColoredPoints(vector<float> &Vertices, std::vector<uint8_t> &Colors);
int ColoredPoints(vector<double> &Vertices, double size_);
int ColoredPoints2(vector<double> &Vertices, vector<double> &Vertices2, double size_);
int ColoredPoints2(vector<double> &Vertices, vector<double> &Vertices2, double size_, vector<orth::Plane> &planes);
int ColoredPoints3(vector<double> &Vertices, vector<double> &Vertices2, vector<double> &Vertices3, double size_);
int ColoredPoints(vector<double> &Vertices, std::vector<uint8_t> &Colors);
void SelectPoints(vector<double> &Vertices);

void ColoredImage3DDistribution(cv::Mat &image_in, double size_);
void Colored2Image3DDistribution(cv::Mat &image_in, cv::Mat &image_in2, double size_);
int ColoredTSDF(vector<float> &Vertices, int resolution_x, int resolution_y, int resolution_z, double size_);

