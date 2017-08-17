
#include <tclap/CmdLine.h>

#include "itkImage.h"
#include "itkNumericTraits.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageSeriesWriter.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkNearestNeighborInterpolateImageFunction.h"

#include <vector>

int main(int argc, char **argv ){

  //Command line parsing
  TCLAP::CmdLine cmd("Derivative Ratio", ' ', "1");

  TCLAP::MultiArg<std::string> imageArg("v","volume","Dicom CT folder", true, "filename");
  cmd.add(imageArg);

  TCLAP::ValueArg<std::string> oArg("o","output","Output filneame", true, "",
      "filename");
  cmd.add(oArg);

  try{
    cmd.parse( argc, argv );
  }
  catch (TCLAP::ArgException &e){
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    return -1;
  }

  typedef float InputPixelType;
  typedef itk::Image< InputPixelType, 3> ImageType;
  typedef itk::NearestNeighborInterpolateImageFunction< ImageType, float > InterpolatorType;

  //Read DICOM CT
  std::vector< ImageType::Pointer > images;
  std::vector< InterpolatorType::Pointer > interpolators;
  std::vector< std::string > folders = imageArg.getValue();
  std::vector< ImageType::IndexType > indices;
  std::vector< ImageType::SizeType > sizes;
  for(int i=0; i < folders.size(); i++){

    typedef itk::ImageSeriesReader< ImageType >     ReaderType;
    typedef itk::GDCMImageIO                        ImageIOType;
    typedef itk::GDCMSeriesFileNames                NamesGeneratorType;

    ImageIOType::Pointer gdcmIO = ImageIOType::New();
    NamesGeneratorType::Pointer namesGenerator = NamesGeneratorType::New();

    namesGenerator->SetInputDirectory( folders[i] );
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

    images.push_back( reader->GetOutput() );
    InterpolatorType::Pointer ip = InterpolatorType::New();
    ip->SetInputImage( reader->GetOutput() );
    interpolators.push_back( ip );

    ImageType::RegionType region = reader->GetOutput()->GetLargestPossibleRegion();
    indices.push_back( region.GetIndex() );
    sizes.push_back( region.GetSize() );
  }

  ImageType::Pointer highRes = ImageType::New();
  ImageType::RegionType highResRegion;
  ImageType::RegionType::SizeType highResSize;
  highResSize.Fill(512);
  highResRegion.SetSize( highResSize );
  ImageType::RegionType::IndexType highResIndex = images[0]->GetLargestPossibleRegion().GetIndex();
  highResRegion.SetIndex( highResIndex );
  ImageType::PointType highResOrigin = images[0]->GetOrigin();
  ImageType::SpacingType highResSpacing;
  ImageType::SpacingType spacing= images[0]->GetSpacing();
  ImageType::SizeType size = images[0]->GetLargestPossibleRegion().GetSize();
  highResSpacing[0] = spacing[0] * size[0] / highResSize[0];
  highResSpacing[1] = spacing[1] * size[1] / highResSize[1];
  highResSpacing[2] = spacing[2] * size[2] / highResSize[2];

  highRes->SetOrigin( highResOrigin );
  highRes->SetSpacing( highResSpacing );
  highRes->SetRegions( highResRegion );
  highRes->SetSpacing( highResSpacing );
  highRes->SetDirection( images[0]->GetDirection() );
  highRes->Allocate();


  typedef itk::ImageRegionIteratorWithIndex< ImageType > IteratorType;

  IteratorType iterator(highRes, highResRegion );
  ImageType::PointType pHighRes;
  ImageType::PointType pImage;
  ImageType::IndexType index;
  ImageType::IndexType minIndex;
  for(iterator.GoToBegin(); !iterator.IsAtEnd(); ++iterator){
    highRes->TransformIndexToPhysicalPoint( iterator.GetIndex(), pHighRes);
    float minD  = itk::NumericTraits<float>::max();
    int minImage = -1;
    for(int i=0; i<images.size(); i++){
      interpolators[i]->ConvertPointToNearestIndex(pHighRes, index);
      for( int k=0; k<3; k++){
        if( index[k] < indices[i][k] ){
          index[k] = indices[i][k];
        }
        else if( index[k] >= indices[i][k] + sizes[i][k] ){
          index[k] = indices[i][k] + sizes[i][k]-1;
        }
      }

      images[i]->TransformIndexToPhysicalPoint(index, pImage);
      float d = pImage.SquaredEuclideanDistanceTo( pHighRes );
      if( d < minD ){
        minIndex = index;
        minImage = i;
        minD = d;
      }
    }
    iterator.Set( images[minImage]->GetPixel( minIndex ) );
  }

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName( oArg.getValue()  );
  writer->SetInput( highRes );
  writer->Update();


  return EXIT_SUCCESS;
}



