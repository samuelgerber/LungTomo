
#include <tclap/CmdLine.h>

#include "itkImage.h"

#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageFileReader.h"
#include "itkImageSeriesReader.h"
#include <rtkThreeDCircularProjectionGeometry.h>
#include "rtkJosephForwardProjectionImageFilter.h"
#include "itkImageFileWriter.h"
#include "itkOrientImageFilter.h"

int main(int argc, char **argv ){

  //Command line parsing
  TCLAP::CmdLine cmd("Derivative Ratio", ' ', "1");

  TCLAP::ValueArg<std::string> imageArg("v","volume","CT Image", true, "",
      "filename");
  cmd.add(imageArg);

  TCLAP::ValueArg<std::string> outputArg("o","ouptu","Output image filename", true, "",
      "filename");
  cmd.add( outputArg );

  TCLAP::SwitchArg dArg( "d", "dicom", "Volume is a dicom folder");
  cmd.add( dArg );

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
  InputImageType::Pointer ct;
  if( dArg.getValue() ){

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
    /*
    for(unsigned int fni = 0; fni < numberOfFileNames; ++fni){
      std::cout << "filename # " << fni << " = ";
      std::cout << filenames[fni] << std::endl;
    }
    */

    ReaderType::Pointer reader = ReaderType::New();
    reader->SetImageIO( gdcmIO );
    reader->SetFileNames( filenames );

    reader->Update();
    ct = reader->GetOutput();

  }
  else{

    typedef itk::ImageFileReader< InputImageType >     ReaderType;
    ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName( imageArg.getValue() );
    reader->Update();
    ct = reader->GetOutput();
  }
  //InputImageType::DirectionType direction = ct->GetDirection();
  //direction.SetIdentity();
  //for(int i=0; i<3; i++){
  //  for(int j=0; j<3; j++){
  //     direction(i, j) = fabs(direction(i,j));
  //  }
  //}
  //direction(2,2) = -1;
  //ct->SetDirection(direction);


  typedef itk::OrientImageFilter<InputImageType,InputImageType> OrientFilter;
  OrientFilter::Pointer orienter = OrientFilter::New();
  orienter->UseImageDirectionOn();
  orienter->SetDesiredCoordinateOrientation(itk::SpatialOrientation::ITK_COORDINATE_ORIENTATION_RIP);
  orienter->SetInput( ct );
  orienter->Update();
  ct = orienter->GetOutput();


  InputImageType::RegionType            ctRegion  = ct->GetLargestPossibleRegion();
  InputImageType::RegionType::SizeType  ctSize    = ctRegion.GetSize();
  InputImageType::RegionType::IndexType ctIndex   = ctRegion.GetIndex();
  InputImageType::PointType             ctOrigin  = ct->GetOrigin();
  InputImageType::SpacingType           ctSpacing = ct->GetSpacing();
  ctOrigin[0] = - (float)ctSize[0] * ctSpacing[0]/2;
  ctOrigin[1] = - (float)ctSize[1] * ctSpacing[1]/2;
  ctOrigin[2] = - (float)ctSize[2] * ctSpacing[2]/2;
  ct->SetOrigin( ctOrigin );

  std::cout << "CT Image" << std::endl;
  std::cout << ctRegion;
  std::cout << "Origin" << std::endl;
  std::cout << ctOrigin << std::endl;
  std::cout << "Spacing" << std::endl;
  std::cout << ctSpacing << std::endl;
  std::cout << "Direction" << std::endl;
  std::cout << ct->GetDirection() << std::endl;



  //Setup forward projection

  int nProjections = 20;

  InputImageType::Pointer projection = InputImageType::New();
  InputImageType::RegionType projectionRegion;
  InputImageType::RegionType::SizeType projectionSize = ctSize;
  projectionSize[0] = 768;
  projectionSize[1] = 768;
  projectionSize[2] = nProjections;
  projectionRegion.SetSize( projectionSize );
  InputImageType::RegionType::IndexType projectionIndex;
  projectionIndex.Fill(0);
  projectionRegion.SetIndex( projectionIndex );
  InputImageType::SpacingType projectionSpacing;
  projectionSpacing = ctSpacing;
  projectionSpacing[0] = ctSpacing[0] * ctSize[0] / projectionSize[0];
  projectionSpacing[1] = ctSpacing[1] * ctSize[1] / projectionSize[1];
  projectionSpacing[2] = 1;
  InputImageType::PointType projectionOrigin = ctOrigin;
  projectionOrigin.Fill(0);
  //projectionOrigin[0] = -projectionSize[0]*projectionSpacing[0]/2;
  //projectionOrigin[1] = -projectionSize[1]*projectionSpacing[1]/2;
  //projectionOrigin[2] = 0;

  projection->SetOrigin(  projectionOrigin  );
  projection->SetSpacing( projectionSpacing );
  projection->SetRegions( projectionRegion  );
  projection->SetSpacing( projectionSpacing );
  projection->Allocate();

  typedef rtk::JosephForwardProjectionImageFilter<InputImageType, OutputImageType> ForwardProjectionType;
  ForwardProjectionType::Pointer forwardProjection = ForwardProjectionType::New();
  forwardProjection->InPlaceOff();
  forwardProjection->SetInput( projection );
  forwardProjection->SetInput( 1, ct );

  typedef rtk::ThreeDCircularProjectionGeometry GeometryType;
  GeometryType::Pointer geometry = GeometryType::New();


  int yStep = 5;
  float xOff = ctOrigin[0] + ctSize[0] * ctSpacing[0]/2;
  float yOff = ctOrigin[1] + ctSize[1] * ctSpacing[1]/2;
  float zOff = 120;
  for( int i=0; i<nProjections; i++){

    geometry->AddProjection(
        zOff,
        ctSize[2]*ctSpacing[2] + zOff+10, 0,
        xOff - projectionSpacing[0] * projectionSize[0]/2,
        yOff - projectionSpacing[1] * projectionSize[1]/2,
        -90, 0,
        xOff ,
        yOff + (i  - (nProjections-1)/2.0 ) * yStep);
  }

  std::cout << geometry << std::endl;
  forwardProjection->SetGeometry( geometry );

  typedef  itk::ImageFileWriter< OutputImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName( outputArg.getValue() );
  writer->SetInput( forwardProjection->GetOutput() );
  writer->Update();

  typedef  itk::ImageFileWriter< InputImageType  > InputWriterType;
  InputWriterType::Pointer iwriter = WriterType::New();
  iwriter->SetFileName( "Input.nrrd" );
  iwriter->SetInput( ct );
  iwriter->Update();

  return EXIT_SUCCESS;
}



