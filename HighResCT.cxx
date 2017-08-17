
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
#include "itkAddImageFilter.h"
#include "itkDivideImageFilter.h"
#include "itkResampleImageFilter.h"

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
  std::vector< std::string > folders = imageArg.getValue();
  ImageType::Pointer highRes;
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
    //for(unsigned int fni = 0; fni < numberOfFileNames; ++fni){
    //  std::cout << "filename # " << fni << " = ";
    //  std::cout << filenames[fni] << std::endl;
    //}

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
    ImageType::Pointer image = reader->GetOutput();

    std::cout << image->GetLargestPossibleRegion();
    std::cout << image->GetSpacing() << std::endl;
    std::cout << image->GetOrigin() << std::endl << std::endl;

    typedef itk::ResampleImageFilter<ImageType, ImageType> ResampleType;
    ResampleType::Pointer resampler = ResampleType::New();
    if( i == 0 ){
      ImageType::RegionType::SizeType highResSize;
      highResSize.Fill(512);
      resampler->SetSize( highResSize );

      ImageType::SpacingType highResSpacing;
      ImageType::SpacingType spacing= image->GetSpacing();
      ImageType::SizeType size = image->GetLargestPossibleRegion().GetSize();
      highResSpacing[0] = spacing[0] * size[0] / highResSize[0];
      highResSpacing[1] = spacing[1] * size[1] / highResSize[1];
      highResSpacing[2] = spacing[2] * size[2] / highResSize[2];
      resampler->SetOutputSpacing( highResSpacing );
      resampler->SetOutputOrigin( image->GetOrigin() );
      resampler->SetOutputDirection( image->GetDirection() );
    }
    else{
      resampler->SetReferenceImage( highRes );
      resampler->UseReferenceImageOn();
    }

    //InterpolatorType::Pointer ip = InterpolatorType::New();
    //resampler->SetInterpolator( ip )

    resampler->SetInput(image);
    resampler->Update();
    highRes = resampler->GetOutput();

    typedef  itk::ImageFileWriter< ImageType  > WriterType;
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName( "tmp.nrrd"  );
    writer->SetInput( highRes );
    writer->Update();

    if( i > 0){
      typedef itk::AddImageFilter<ImageType, ImageType, ImageType> AddImageFilter;
      AddImageFilter::Pointer add = AddImageFilter::New();
      add->SetInput1( highRes );
      add->SetInput2( resampler->GetOutput() );
      add->Update();
      highRes = add->GetOutput();
    }
  }


  typedef itk::DivideImageFilter< ImageType, ImageType, ImageType> DivideImageFilter;
  DivideImageFilter::Pointer divide = DivideImageFilter::New();
  divide->SetInput1( highRes );
  divide->SetConstant( folders.size() );
  divide->Update();
  highRes = divide->GetOutput();

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName( oArg.getValue()  );
  writer->SetInput( highRes );
  writer->Update();


  return EXIT_SUCCESS;
}



