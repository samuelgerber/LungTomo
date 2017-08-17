
#include <tclap/CmdLine.h>

#include "itkImage.h"

#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageSeriesWriter.h"
#include <rtkThreeDCircularProjectionGeometry.h>
#include "rtkJosephForwardProjectionImageFilter.h"
#include "itkImageFileWriter.h"
#include "itkOrientImageFilter.h"

int main(int argc, char **argv ){

  //Command line parsing
  TCLAP::CmdLine cmd("Derivative Ratio", ' ', "1");

  TCLAP::ValueArg<std::string> imageArg("v","volume","Dicom CT folder", true, "",
      "filename");
  cmd.add(imageArg);

  TCLAP::ValueArg<std::string> prefixArg("p","prefix","Prefix for storing output images", true, "",
      "filename");
  cmd.add(prefixArg);

  try{
    cmd.parse( argc, argv );
  }
  catch (TCLAP::ArgException &e){
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    return -1;
  }

  typedef float InputPixelType;
  typedef itk::Image< InputPixelType, 3> InputImageType;
  typedef float OutputPixelType;
  typedef itk::Image< OutputPixelType, 3> OutputImageType;

  //Read DICOM CT

  typedef itk::ImageSeriesReader< InputImageType >     ReaderType;
  typedef itk::GDCMImageIO                        ImageIOType;
  typedef itk::GDCMSeriesFileNames                NamesGeneratorType;

  ImageIOType::Pointer gdcmIO = ImageIOType::New();
  NamesGeneratorType::Pointer namesGenerator = NamesGeneratorType::New();

  namesGenerator->SetInputDirectory( imageArg.getValue() );
  const ReaderType::FileNamesContainer & filenames =
                            namesGenerator->GetInputFileNames();

  std::size_t numberOfFileNames = filenames.size();
  std::cout << numberOfFileNames << std::endl;
  for(unsigned int fni = 0; fni < numberOfFileNames; ++fni){
    std::cout << "filename # " << fni << " = ";
    std::cout << filenames[fni] << std::endl;
  }

  ReaderType::Pointer reader = ReaderType::New();
  reader->SetImageIO( gdcmIO );
  reader->SetFileNames( filenames );

  try{
    reader->Update();
  }
  catch (itk::ExceptionObject &excp){
    std::cerr << "Exception thrown while writing the image" << std::endl;
    std::cerr << excp << std::endl;
    return EXIT_FAILURE;
  }

  InputImageType::Pointer ct = reader->GetOutput();
  InputImageType::DirectionType direction = ct->GetDirection();
  direction.SetIdentity();
  ct->SetDirection(direction);

  /*
  typedef itk::OrientImageFilter<InputImageType,InputImageType> OrientFilter;
  OrientFilter::Pointer orienter = OrientFilter::New();
  orienter->UseImageDirectionOn();
  orienter->SetDesiredCoordinateOrientation(itk::SpatialOrientation::ITK_COORDINATE_ORIENTATION_LSA);//ITK_COORDINATE_ORIENTATION_RIP);
  orienter->SetInput( ct );
  orienter->Update();
  ct = orienter->GetOutput();
  */

  InputImageType::RegionType            ctRegion  = ct->GetLargestPossibleRegion();
  InputImageType::RegionType::SizeType  ctSize    = ctRegion.GetSize();
  InputImageType::RegionType::IndexType ctIndex   = ctRegion.GetIndex();
  InputImageType::PointType             ctOrigin  = ct->GetOrigin();
  InputImageType::SpacingType           ctSpacing = ct->GetSpacing();

  std::cout << "CT Image" << std::endl;
  std::cout << ctRegion;
  std::cout << "Origin" << std::endl;
  std::cout << ctOrigin << std::endl;
  std::cout << "Spacing" << std::endl;
  std::cout << ctSpacing << std::endl;
  std::cout << "Direction" << std::endl;
  std::cout << ct->GetDirection() << std::endl;



  //Setup forward projection


  InputImageType::Pointer projection = InputImageType::New();
  InputImageType::RegionType projectionRegion;
  InputImageType::RegionType::SizeType projectionSize = ctSize;
  projectionSize[0] = 2560;
  projectionSize[1] = 3072;
  projectionSize[2] = 1;
  projectionRegion.SetSize( projectionSize );
  InputImageType::RegionType::IndexType projectionIndex;
  projectionIndex.Fill(0);
  projectionRegion.SetIndex( projectionIndex );
  InputImageType::PointType projectionOrigin = ctOrigin;
  projectionOrigin.Fill(0);
  //projectionOrigin[0] -= ctSize[0]*ctSpacing[0];
  //projectionOrigin[1] = ctOrigin[2] - ctSize[2]*ctSpacing[2];
  //projectionOrigin[2] -= ctSize[2]*ctSpacing[2];
  InputImageType::SpacingType projectionSpacing;
  projectionSpacing = ctSpacing;
  projectionSpacing[0] = ctSpacing[0] * ctSize[0] / projectionSize[0];
  projectionSpacing[1] = ctSpacing[1] * ctSize[1] / projectionSize[1];
  projectionSpacing[2] = 1;

  projection->SetOrigin( projectionOrigin );
  projection->SetSpacing( projectionSpacing );
  projection->SetRegions( projectionRegion );
  projection->SetSpacing( projectionSpacing );
  projection->Allocate();

  typedef rtk::JosephForwardProjectionImageFilter<InputImageType, OutputImageType> ForwardProjectionType;
  ForwardProjectionType::Pointer forwardProjection = ForwardProjectionType::New();
  forwardProjection->InPlaceOff();
  forwardProjection->SetInput( projection );
  forwardProjection->SetInput( 1, ct );

  typedef rtk::ThreeDCircularProjectionGeometry GeometryType;
  GeometryType::Pointer geometry = GeometryType::New();

  /*
  GeometryType::PointType sourcePosition = ctOrigin;
  sourcePosition[0] += ctSize[0] * ctSpacing[0] / 2;
  sourcePosition[1] += ctSize[1] * 4*ctSpacing[1];
  sourcePosition[2] += ctSize[2] * ctSpacing[2] / 2;
  GeometryType::PointType detectorPosition = ctOrigin;
  //detectorPosition[0] += ctSize[0] * ctSpacing[0] ;
  //detectorPosition[1] += ctSize[1] * 4*ctSpacing[1];
  //detectorPosition[2] += ctSize[2] * ctSpacing[2] ;
  GeometryType::VectorType detectorRow;
  detectorRow.Fill(0);
  detectorRow[0] = 1;
  GeometryType::VectorType detectorCol;
  detectorCol.Fill(0);
  detectorCol[2] = -1;
  */


  geometry->AddProjection(
        137 + ctSize[2]*ctSpacing[2] * 6,
        137 + ctSize[2]*ctSpacing[2] * 7,
        0,
        ctOrigin[0] + ctSize[0] * ctSpacing[0]/2 - projectionSpacing[0] * projectionSize[0]/2,
        ctOrigin[1] + ctSize[1] * ctSpacing[1]/2 - projectionSpacing[1] * projectionSize[1]/2,
        0, 0,
        ctOrigin[0] + ctSize[0]*ctSpacing[0]/2,
        ctOrigin[1] + ctSize[1]*ctSpacing[1]/2);
  std::cout << geometry << std::endl;
  forwardProjection->SetGeometry( geometry );

  typedef  itk::ImageFileWriter< OutputImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName( "projection.nrrd" );
  writer->SetInput( forwardProjection->GetOutput() );
  writer->Update();

  typedef  itk::ImageFileWriter< InputImageType  > InputWriterType;
  InputWriterType::Pointer iwriter = WriterType::New();
  iwriter->SetFileName( "Input.nrrd" );
  iwriter->SetInput( ct );
  iwriter->Update();

  return EXIT_SUCCESS;
}



