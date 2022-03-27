#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <gmpxx.h>
#include <gmp.h>
#include <cmath>

#include "vo.hpp"

using namespace cv;
using namespace std;
//
const int sign = 1;
//const int bit_width = 60; //have to less than FIXP_INT_SCALAR_TYPE? Can't use 64 for 1LL
//const int shift = 14;
const int bit_width = 63; //have to less than FIXP_INT_SCALAR_TYPE? Can't use 64 for 1LL
const int shift = 12;
const FixedPointConfig fpconfig(sign, bit_width, shift);
//

const int maxLineDiff = 70;
const int sobelSize = 3;
const float sobelScale = 1./8.;

//const float iterCounts = 7;
static inline
void setDefaultIterCounts(Mat& iterCounts)
{
    iterCounts = Mat(Vec4i(7,7,7,10));
    //iterCounts = Mat(Vec4i(7));
}

const float minGradientMagnitudes = 10;
/*
static inline
void setDefaultMinGradientMagnitudes(Mat& minGradientMagnitudes)
{
    minGradientMagnitudes = Mat(Vec4f(10,10,10,10));
}
*/
/*
static
void normalsComputer(const Mat& points3d, int rows, int cols, Mat & normals) 
{
  normals.create(points3d.size(), CV_MAKETYPE(points3d.depth(), 3));
  for (int y = 0; y < rows - 1; ++y)
  {
    for (int x = 0; x < cols - 1; ++x)
    {
    	Vec3f du = points3d.at<Vec3f>(y,x+1) - points3d.at<Vec3f>(y,x);
    	Vec3f dv = points3d.at<Vec3f>(y+1,x) - points3d.at<Vec3f>(y,x);
        normals.at<Vec3f>(y,x) = du.cross(dv);
    	float norm = sqrt(normals.at<Vec3f>(y,x)[0]*normals.at<Vec3f>(y,x)[0] + normals.at<Vec3f>(y,x)[1]*normals.at<Vec3f>(y,x)[1] +normals.at<Vec3f>(y,x)[2]*normals.at<Vec3f>(y,x)[2]);
            normals.at<Vec3f>(y,x)[0] = normals.at<Vec3f>(y,x)[0] / norm;
            normals.at<Vec3f>(y,x)[1] = normals.at<Vec3f>(y,x)[1] / norm;
            normals.at<Vec3f>(y,x)[2] = normals.at<Vec3f>(y,x)[2] / norm;
    }
  }
}
*/
static
vector<FixedPointVector> normalsComputer(vector<FixedPointVector>& p3d_vec, int rows, int cols) 
{
  FixedPointScalar zero_value((FIXP_SCALAR_TYPE)0, fpconfig);
  FixedPointVector zero_vec(zero_value, zero_value, zero_value);
  vector<FixedPointVector> normals(rows*cols, zero_vec);
  for (int y = 0; y < rows - 1; ++y)
  {
    for (int x = 0; x < cols - 1; ++x)
    {
            FixedPointScalar du_x = p3d_vec[y*cols + (x+1)].x - p3d_vec[y*cols + x].x;
            FixedPointScalar dv_x = p3d_vec[(y+1)*cols + x].x - p3d_vec[y*cols + x].x;
            FixedPointScalar du_y = p3d_vec[y*cols + (x+1)].y - p3d_vec[y*cols + x].y;
            FixedPointScalar dv_y = p3d_vec[(y+1)*cols + x].y - p3d_vec[y*cols + x].y;
            FixedPointScalar du_z = p3d_vec[y*cols + (x+1)].z - p3d_vec[y*cols + x].z;
            FixedPointScalar dv_z = p3d_vec[(y+1)*cols + x].z - p3d_vec[y*cols + x].z;
            FixedPointScalar n_x = (du_y * dv_z) - (du_z * dv_y);
            FixedPointScalar n_y = (du_z * dv_x) - (du_x * dv_z);
            FixedPointScalar n_z = (du_x * dv_y) - (du_y * dv_x);
            FixedPointScalar n2_x = n_x*n_x;
            FixedPointScalar n2_y = n_y*n_y;
            FixedPointScalar n2_z = n_z*n_z;
            FixedPointScalar norm_pre = n2_x + n2_y + n2_z;
            FixedPointScalar norm = (norm_pre).sqrt();
            if(!(mpz_get_si(norm.big_value)==0))
            {
                FixedPointScalar n_x_final = n_x / norm;
                FixedPointScalar n_y_final = n_y / norm;
                FixedPointScalar n_z_final = n_z / norm;
                FixedPointVector normal(n_x_final, n_y_final, n_z_final);   
                normals[y*cols + x] = normal;
            }
    }
  }
  return normals;
}


RgbdFrame::RgbdFrame() : ID(-1)
{}

RgbdFrame::RgbdFrame(const Mat& image_in, const Mat& depth_in, const Mat& mask_in, const Mat& normals_in, int ID_in)
    : ID(ID_in), image(image_in), depth(depth_in), mask(mask_in), normals(normals_in)
{}

RgbdFrame::~RgbdFrame()
{}

void RgbdFrame::release()
{
    ID = -1;
    image.release();
    depth.release();
    mask.release();
    normals.release();
    cloud.release();
}

OdometryFrame::OdometryFrame() : RgbdFrame()
{}

OdometryFrame::OdometryFrame(const Mat& image_in, const Mat& depth_in, const Mat& mask_in, const Mat& normals_in, int ID_in)
    : RgbdFrame(image_in, depth_in, mask_in, normals_in, ID_in)
{}

void OdometryFrame::release()
{
    RgbdFrame::release();
    releasePyramids();
}

void OdometryFrame::releasePyramids()
{
    dI_dx.release();
    dI_dy.release();
    maskText.release();
    maskDepth.release();
    maskNormal.release();
}


Odometry::Odometry() :
    minDepth(DEFAULT_MIN_DEPTH()),
    maxDepth(DEFAULT_MAX_DEPTH()),
    maxDepthDiff(DEFAULT_MAX_DEPTH_DIFF()),
    maxPointsPart(DEFAULT_MAX_POINTS_PART()),
    maxTranslation(DEFAULT_MAX_TRANSLATION()),
    maxRotation(DEFAULT_MAX_ROTATION())

{
    setDefaultIterCounts(iterCounts);
    //setDefaultMinGradientMagnitudes(minGradientMagnitudes);
}

Odometry::Odometry(const Mat& _cameraMatrix,
                   float _minDepth, float _maxDepth, float _maxDepthDiff,
                   const std::vector<int>& _iterCounts,
                   /*const std::vector<float>& _minGradientMagnitudes,*/
                   float _maxPointsPart) :
                   minDepth(_minDepth), maxDepth(_maxDepth), maxDepthDiff(_maxDepthDiff),
                   iterCounts(Mat(_iterCounts).clone()),
                   //minGradientMagnitudes(Mat(_minGradientMagnitudes).clone()),
                   maxPointsPart(_maxPointsPart),
                   cameraMatrix(_cameraMatrix),
                   maxTranslation(DEFAULT_MAX_TRANSLATION()), maxRotation(DEFAULT_MAX_ROTATION())
{
    if(iterCounts.empty() /*|| minGradientMagnitudes.empty()*/)
    {
        setDefaultIterCounts(iterCounts);
        //setDefaultMinGradientMagnitudes(minGradientMagnitudes);
    }
}

inline void
rescaleDepth(InputArray in_in, int depth, OutputArray out_out)
{
  cv::Mat in = in_in.getMat();
  CV_Assert(in.type() == CV_64FC1 || in.type() == CV_32FC1 || in.type() == CV_16UC1 || in.type() == CV_16SC1);
  CV_Assert(depth == CV_64FC1 || depth == CV_32FC1);

  int in_depth = in.depth();

  out_out.create(in.size(), depth);
  cv::Mat out = out_out.getMat();
  if (in_depth == CV_16U)
  {
    in.convertTo(out, depth, 1 / 1000.0); //convert to float so that it is in meters
    cv::Mat valid_mask = in == std::numeric_limits<ushort>::min(); // Should we do std::numeric_limits<ushort>::max() too ?
    out.setTo(std::numeric_limits<float>::quiet_NaN(), valid_mask); //set a$
  }
  if (in_depth == CV_16S)
  {
    in.convertTo(out, depth, 1 / 1000.0); //convert to float so tha$
    cv::Mat valid_mask = (in == std::numeric_limits<short>::min()) | (in == std::numeric_limits<short>::max()); // Should we do std::numeric_limits<ushort>::max() too ?
    out.setTo(std::numeric_limits<float>::quiet_NaN(), valid_mask); //set a$
  }
  if ((in_depth == CV_32F) || (in_depth == CV_64F))
    in.convertTo(out, depth);
}

template<typename T>
void
rescaleDepthTemplated(const Mat& in, Mat& out);

template<>
inline void
rescaleDepthTemplated<float>(const Mat& in, Mat& out)
{
  rescaleDepth(in, CV_32F, out);
}

template<typename T>
void
depthTo3dNoMask(const cv::Mat& in_depth, const cv::Mat_<T>& K, cv::Mat& points3d)
{
  const T inv_fx = T(1) / K(0, 0);
  const T inv_fy = T(1) / K(1, 1);
  const T ox = K(0, 2);
  const T oy = K(1, 2);

  // Build z
  cv::Mat_<T> z_mat;
  cout << "liyang test" << in_depth.depth() << endl;
  cout << "liyang test" << z_mat.depth() << endl;
  exit(1);
  if (z_mat.depth() == in_depth.depth())
    z_mat = in_depth;
  else
    rescaleDepthTemplated<T>(in_depth, z_mat);

  // Pre-compute some constants
  cv::Mat_<T> x_cache(1, in_depth.cols), y_cache(in_depth.rows, 1);
  T* x_cache_ptr = x_cache[0], *y_cache_ptr = y_cache[0];
  for (int x = 0; x < in_depth.cols; ++x, ++x_cache_ptr)
    *x_cache_ptr = (x - ox) * inv_fx;
  for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
    *y_cache_ptr = (y - oy) * inv_fy;

  y_cache_ptr = y_cache[0];
  for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
  {
    cv::Vec<T, 3>* point = points3d.ptr<cv::Vec<T, 3> >(y);
    const T* x_cache_ptr_end = x_cache[0] + in_depth.cols;
    const T* depth = z_mat[y];
    for (x_cache_ptr = x_cache[0]; x_cache_ptr != x_cache_ptr_end; ++x_cache_ptr, ++point, ++depth)
    {
      T z = *depth;
      (*point)[0] = (*x_cache_ptr) * z;
      (*point)[1] = (*y_cache_ptr) * z;
      (*point)[2] = z;
      //cout << "liyang test" << (*point)[0] << endl;
      //cout << "liyang test" << (*point)[1] << endl;
      //cout << "liyang test" << (*point)[2] << endl;
      //exit(1);
    }
  }
}

vector<FixedPointVector> depthTo3d(const vector<FixedPointScalar>& depth, const Mat& K, int rows, int cols)
{
  vector<FixedPointScalar> cam_fixp;
  cam_fixp = f_Mat2Vec(K, fpconfig);
  FixedPointScalar fx =cam_fixp[0];
  FixedPointScalar fy =cam_fixp[4];
  FixedPointScalar cx =cam_fixp[2];
  FixedPointScalar cy =cam_fixp[5];
  vector<FixedPointScalar> depth_vec;

  // Create 3D points in one go.
  vector<FixedPointVector> p3d_vec;
  vector<FixedPointScalar> x_cache;
  vector<FixedPointScalar> y_cache;
  for (int x = 0; x < cols; ++x)
  {
    FixedPointScalar p_x((FIXP_SCALAR_TYPE)x, fpconfig);
     x_cache.push_back((p_x - cx) / fx);
  }
  for (int y = 0; y < rows; ++y)
  {
    FixedPointScalar p_y((FIXP_SCALAR_TYPE)y, fpconfig);
     y_cache.push_back((p_y - cy) / fy);
  }
  for (int y = 0; y < rows; ++y)
  {
    for (int x = 0; x < cols; ++x)
    {
         FixedPointVector p3d(x_cache[x]*depth[y*cols + x] , y_cache[y]*depth[y*cols + x], depth[y*cols + x]);
         p3d_vec.push_back(p3d);
    }
  }

  return p3d_vec;
}

/*
void
depthTo3d(InputArray depth_in, InputArray K_in, OutputArray points3d_out)
{
  cv::Mat depth = depth_in.getMat();
  cv::Mat K = K_in.getMat();
  CV_Assert(K.cols == 3 && K.rows == 3 && (K.depth() == CV_64F || K.depth()==CV_32F));
  CV_Assert(
      depth.type() == CV_64FC1 || depth.type() == CV_32FC1 || depth.type() == CV_16UC1 || depth.type() == CV_16SC1);

  // TODO figure out what to do when types are different: convert or reject ?
  cv::Mat K_new;
  if ((depth.depth() == CV_32F || depth.depth() == CV_64F) && depth.depth() != K.depth())
  {
    K.convertTo(K_new, depth.depth());
  }
  else
    K_new = K;

  // Create 3D points in one go.
  points3d_out.create(depth.size(), CV_MAKETYPE(K_new.depth(), 3));
  cv::Mat points3d = points3d_out.getMat();
  depthTo3dNoMask<float>(depth, K_new, points3d);
}
*/
/*
depthTo3d(const cv::Mat& in_depth, const cv::Mat_<float>& K, cv::Mat& points3d)
{
  points3d.create(in_depth.size(), CV_MAKETYPE(in_depth.depth(), 3));
  const T inv_fx = T(1) / K(0, 0);
  const T inv_fy = T(1) / K(1, 1);
  const T ox = K(0, 2);
  const T oy = K(1, 2);

  FixedPointScalar 

}
*/
static
void randomSubsetOfMask(Mat& mask, float part)
{
    const int minPointsCount = 1000; // minimum point count (we can process them fast)
    const int nonzeros = countNonZero(mask);
    const int needCount = std::max(minPointsCount, int(mask.total() * part));
    if(needCount < nonzeros)
    {
        RNG rng;
        Mat subset(mask.size(), CV_8UC1, Scalar(0));

        int subsetSize = 0;
        while(subsetSize < needCount)
        {
            int y = rng(mask.rows);
            int x = rng(mask.cols);
            if(mask.at<uchar>(y,x))
            {
                subset.at<uchar>(y,x) = 255;
                mask.at<uchar>(y,x) = 0;
                subsetSize++;
            }
        }
        mask = subset;
    }
}

static inline
void checkImage(const Mat& image)
{
    if(image.empty())
        CV_Error(Error::StsBadSize, "Image is empty.");
    if(image.type() != CV_8UC1)
        CV_Error(Error::StsBadSize, "Image type has to be CV_8UC1.");
}

static inline
void checkDepth(const Mat& depth, const Size& imageSize)
{
    if(depth.empty())
        CV_Error(Error::StsBadSize, "Depth is empty.");
    if(depth.size() != imageSize)
        CV_Error(Error::StsBadSize, "Depth has to have the size equal to the image size.");
    if(depth.type() != CV_32FC1)
        CV_Error(Error::StsBadSize, "Depth type has to be CV_32FC1.");
    //if(depth.type() != CV_32SC1)
    //    CV_Error(Error::StsBadSize, "Depth type has to be CV_32SC1.");
}

static inline
void checkMask(const Mat& mask, const Size& imageSize)
{
    if(!mask.empty())
    {
        if(mask.size() != imageSize)
            CV_Error(Error::StsBadSize, "Mask has to have the size equal to the image size.");
        if(mask.type() != CV_8UC1)
            CV_Error(Error::StsBadSize, "Mask type has to be CV_8UC1.");
    }
}

static inline
void checkNormals(const Mat& normals, const Size& depthSize)
{
    if(normals.size() != depthSize)
        CV_Error(Error::StsBadSize, "Normals has to have the size equal to the depth size.");
    if(normals.type() != CV_32FC3)
        CV_Error(Error::StsBadSize, "Normals type has to be CV_32FC3.");
}

static
void MaskGen(const Mat& mask, const Mat& Depth, float minDepth, float maxDepth,
                        const Mat& Normal,
                        Mat& maskDepth)
{
    minDepth = std::max(0.f, minDepth);

    if(!maskDepth.empty())
    {
        CV_Assert(maskDepth.size() == Depth.size());
        CV_Assert(maskDepth.type() == CV_8UC1);
    }
    else
    {
        //Mat maskDepth;
        if(mask.empty())
            maskDepth = Mat(Depth.size(), CV_8UC1, Scalar(255));
        else
            maskDepth = mask.clone();

        Mat levelDepth = Depth.clone();
        patchNaNs(levelDepth, 0);

        maskDepth &= (levelDepth > minDepth) & (levelDepth < maxDepth);

        //if(!Normal.empty())
        //{
        //    Mat validNormalMask = Normal == Normal; // otherwise it's Nan
        //    
        //    CV_Assert(validNormalMask.type() == CV_8UC3);

        //    std::vector<Mat> channelMasks;
        //    split(validNormalMask, channelMasks);
        //    validNormalMask = channelMasks[0] & channelMasks[1] & channelMasks[2];

        //    maskDepth &= validNormalMask;
        //}
    }
}

static
void TexturedMaskGen(const Mat& dI_dx, const Mat& dI_dy,
                     const float& minGradientMagnitudes, const Mat& Mask, float maxPointsPart,
                     Mat& texturedMask)
{
    if(!texturedMask.empty())
    {
        CV_Assert(texturedMask.size() == dI_dx.size());
        CV_Assert(texturedMask.type() == CV_8UC1);
    }
    else
    {
        const float sobelScale2_inv = 1.f / (float)(sobelScale * sobelScale);
        const float minScaledGradMagnitude2 = minGradientMagnitudes * minGradientMagnitudes * sobelScale2_inv;
        Mat texturedMask_pre(dI_dx.size(), CV_8UC1, Scalar(0));
        for(int y = 0; y < dI_dx.rows; y++)
        {
            const short *dIdx_row = dI_dx.ptr<short>(y);
            const short *dIdy_row = dI_dy.ptr<short>(y);
            uchar *texturedMask_row = texturedMask_pre.ptr<uchar>(y);
            for(int x = 0; x < dI_dx.cols; x++)
            {
                float magnitude2 = static_cast<float>(dIdx_row[x] * dIdx_row[x] + dIdy_row[x] * dIdy_row[x]);
                if(magnitude2 >= minScaledGradMagnitude2)
                    texturedMask_row[x] = 255;
            }
        }
        texturedMask = texturedMask_pre & Mask;

        //randomSubsetOfMask(texturedMask, (float)maxPointsPart);
    }
}

void NormalsMaskGen(const Mat& normals, const Mat& Mask, float maxPointsPart,
                    Mat& maskNormal)
{
    if(!maskNormal.empty())
    {
        CV_Assert(maskNormal.size() == Mask.size());
        CV_Assert(maskNormal.type() == Mask.type());
    }
    else
    {
        maskNormal = Mask.clone();
        //for(int y = 0; y < maskNormal.rows; y++)
        //{
        //    const Vec3f *normals_row = normals.ptr<Vec3f>(y);
        //    uchar *normalsMask_row = maskNormal.ptr<uchar>(y);
        //    for(int x = 0; x < maskNormal.cols; x++)
        //    {
        //        Vec3f n = normals_row[x];
        //        if(cvIsNaN(n[0]))
        //        {
        //            CV_DbgAssert(cvIsNaN(n[1]) && cvIsNaN(n[2]));
        //            normalsMask_row[x] = 0;
        //        }
        //    }
        //}
        //randomSubsetOfMask(maskNormal, (float)maxPointsPart);
    }
}


Size Odometry::prepareFrameCache(Ptr<OdometryFrame>& frame, int cacheType) const
{
    
    if(frame->normals.empty())
    {
    checkImage(frame->image);

    checkDepth(frame->depth, frame->image.size());
    
    checkMask(frame->mask, frame->image.size());
   
    //depthTo3d(frame->depth, cameraMatrix, frame->cloud);
    //std::cout << frame->depth.at<float>(0,0) <<  std::endl;
    frame->depth_vec = f_Mat2Vec(frame->depth, fpconfig);
    frame->cloud_vec = depthTo3d(frame->depth_vec, cameraMatrix, frame->depth.rows, frame->depth.cols);
    frame->cloud = PVec2Mat_f(frame->cloud_vec, frame->depth.rows, frame->depth.cols);

    //if(cacheType & OdometryFrame::CACHE_DST)
    //{
    //    if(frame->normals.empty())
    //    {
    //            //normalsComputer(frame->cloud, frame->depth.rows, frame->depth.cols, frame->normals);
    //            vector<FixedPointVector> normals_vec = normalsComputer(frame->cloud_vec, frame->depth.rows, frame->depth.cols);
    //            frame->normals = PVec2Mat_f(normals_vec, frame->depth.rows, frame->depth.cols);
    //    }
    //    checkNormals(frame->normals, frame->depth.size());

    //    MaskGen(frame->mask, frame->depth, (float)minDepth, (float)maxDepth,
    //            frame->normals, frame->maskDepth);

    //    Sobel(frame->image, frame->dI_dx, CV_16S, 1, 0, sobelSize);
    //    Sobel(frame->image, frame->dI_dy, CV_16S, 0, 1, sobelSize);

    //    TexturedMaskGen(frame->dI_dx, frame->dI_dy,
    //                    minGradientMagnitudes, frame->maskDepth,
    //                    maxPointsPart, frame->maskText);

    //    NormalsMaskGen(frame->normals, frame->maskDepth, maxPointsPart, frame->maskNormal);
    //}
    //else
    //    MaskGen(frame->mask, frame->depth, (float)minDepth, (float)maxDepth,
    //            frame->normals, frame->maskDepth);

    frame->normals_vec = normalsComputer(frame->cloud_vec, frame->depth.rows, frame->depth.cols);
    frame->normals = PVec2Mat_f(frame->normals_vec, frame->depth.rows, frame->depth.cols);

    MaskGen(frame->mask, frame->depth, (float)minDepth, (float)maxDepth,
            frame->normals, frame->maskDepth);

    Sobel(frame->image, frame->dI_dx, CV_16S, 1, 0, sobelSize);
    Sobel(frame->image, frame->dI_dy, CV_16S, 0, 1, sobelSize);

    TexturedMaskGen(frame->dI_dx, frame->dI_dy,
                    minGradientMagnitudes, frame->maskDepth,
                    maxPointsPart, frame->maskText);

    NormalsMaskGen(frame->normals, frame->maskDepth, maxPointsPart, frame->maskNormal);
    }
    return frame->image.size();
}

/*
static
int computeCorresps(const Mat& K, const Mat& K_inv, const Mat& Rt,
                     const Mat& depth0, const Mat& validMask0,
                     const Mat& depth1, const Mat& selectMask1, float maxDepthDiff,
                     Mat& _corresps)
{
    Mat corresps(depth1.size(), CV_16SC2, Scalar::all(-1));

    Rect r(0, 0, depth1.cols, depth1.rows);
    Mat Kt = Rt(Rect(3,0,1,3)).clone();
    Kt = K * Kt;
    const float * Kt_ptr = Kt.ptr<const float>();

    AutoBuffer<float> buf(3 * (depth1.cols + depth1.rows));
    float *KRK_inv0_u1 = buf;
    float *KRK_inv1_v1_plus_KRK_inv2 = KRK_inv0_u1 + depth1.cols;
    float *KRK_inv3_u1 = KRK_inv1_v1_plus_KRK_inv2 + depth1.rows;
    float *KRK_inv4_v1_plus_KRK_inv5 = KRK_inv3_u1 + depth1.cols;
    float *KRK_inv6_u1 = KRK_inv4_v1_plus_KRK_inv5 + depth1.rows;
    float *KRK_inv7_v1_plus_KRK_inv8 = KRK_inv6_u1 + depth1.cols;
    {
        Mat R = Rt(Rect(0,0,3,3)).clone();

        Mat KRK_inv = K * R * K_inv;
        const float * KRK_inv_ptr = KRK_inv.ptr<const float>();
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            KRK_inv0_u1[u1] = (KRK_inv_ptr[0] * u1);
            KRK_inv3_u1[u1] = (KRK_inv_ptr[3] * u1);
            KRK_inv6_u1[u1] = (KRK_inv_ptr[6] * u1);
        }

        for(int v1 = 0; v1 < depth1.rows; v1++)
        {
            KRK_inv1_v1_plus_KRK_inv2[v1] = (KRK_inv_ptr[1] * v1 + KRK_inv_ptr[2]);
            KRK_inv4_v1_plus_KRK_inv5[v1] = (KRK_inv_ptr[4] * v1 + KRK_inv_ptr[5]);
            KRK_inv7_v1_plus_KRK_inv8[v1] = (KRK_inv_ptr[7] * v1 + KRK_inv_ptr[8]);
        }
    }

    int correspCount = 0;
    for(int v1 = 0; v1 < depth1.rows; v1++)
    {
        const float *depth1_row = depth1.ptr<float>(v1);
        const uchar *mask1_row = selectMask1.ptr<uchar>(v1);
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            float d1 = depth1_row[u1];
            if(mask1_row[u1])
            {
                CV_DbgAssert(!cvIsNaN(d1));
                float transformed_d1 = static_cast<float>(d1 * (KRK_inv6_u1[u1] + KRK_inv7_v1_plus_KRK_inv8[v1]) +
                                                          Kt_ptr[2]);
                if(transformed_d1 > 0)
                {
                    float transformed_d1_inv = 1.f / transformed_d1;
                    int u0 = cvRound(transformed_d1_inv * (d1 * (KRK_inv0_u1[u1] + KRK_inv1_v1_plus_KRK_inv2[v1]) +
                                                           Kt_ptr[0]));
                    int v0 = cvRound(transformed_d1_inv * (d1 * (KRK_inv3_u1[u1] + KRK_inv4_v1_plus_KRK_inv5[v1]) +
                                                           Kt_ptr[1]));

                    if(r.contains(Point(u0,v0)))
                    {
                        float d0 = depth0.at<float>(v0,u0);
                        if(validMask0.at<uchar>(v0, u0) && std::abs(transformed_d1 - d0) <= maxDepthDiff && std::abs(v1 - v0) <= maxLineDiff)
                        {
                            CV_DbgAssert(!cvIsNaN(d0));
                            Vec2s& c = corresps.at<Vec2s>(v0,u0);
                            if(c[0] != -1)
                            {
                                int exist_u1 = c[0], exist_v1 = c[1];

                                float exist_d1 = (float)(depth1.at<float>(exist_v1,exist_u1) *
                                    (KRK_inv6_u1[exist_u1] + KRK_inv7_v1_plus_KRK_inv8[exist_v1]) + Kt_ptr[2]);

                                if(transformed_d1 > exist_d1)
                                    continue;
                            }
                            else
                                correspCount++;

                            c = Vec2s((short)u1, (short)v1);
                        }
                    }
                }
            }
        }
    }

    int v_max_corr = 0;
    _corresps.create(correspCount, 1, CV_32SC4);
    Vec4i * corresps_ptr = _corresps.ptr<Vec4i>();
    for(int v0 = 0, i = 0; v0 < corresps.rows; v0++)
    {
        const Vec2s* corresps_row = corresps.ptr<Vec2s>(v0);
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            const Vec2s& c = corresps_row[u0];
            if(c[0] != -1)
            {
                corresps_ptr[i++] = Vec4i(u0,v0,c[0],c[1]);
                int v_diff = abs(v0-c[1]);
                if(v_diff > v_max_corr)
                    v_max_corr = v_diff;
            }
        }
    }
    return v_max_corr;
}
*/
static
int  computeCorresps(const Mat& K, const Mat& K_inv, const Mat& Rt,
                     const vector<FixedPointScalar>& d0_vec, const Mat& validMask0,
                     const vector<FixedPointScalar>& d1_vec, const Mat& selectMask1, float maxDepthDiff, int rows, int cols,
                     Mat& _corresps)
{

  FixedPointScalar fx (K.at<float>(0,0), fpconfig);//float
  FixedPointScalar fy (K.at<float>(1,1), fpconfig);//float
  FixedPointScalar cx (K.at<float>(0,2), fpconfig);//float
  FixedPointScalar cy (K.at<float>(1,2), fpconfig);//float
  FixedPointScalar fx_inv (K_inv.at<float>(0,0), fpconfig);//float
  FixedPointScalar fy_inv (K_inv.at<float>(1,1), fpconfig);//float
  FixedPointScalar cx_inv (K_inv.at<float>(0,2), fpconfig);//float
  FixedPointScalar cy_inv (K_inv.at<float>(1,2), fpconfig);//float

  vector<FixedPointScalar> Rt_vec;
  Rt_vec = f_Mat2Vec(Rt, fpconfig);

  FixedPointScalar RK_inv_00 = Rt_vec[0]*fx_inv;
  FixedPointScalar RK_inv_01 = Rt_vec[1]*fy_inv;
  FixedPointScalar RK_inv_02 = Rt_vec[0]*cx_inv + Rt_vec[1]*cy_inv + Rt_vec[2];
  FixedPointScalar RK_inv_10 = Rt_vec[4]*fx_inv;
  FixedPointScalar RK_inv_11 = Rt_vec[5]*fy_inv;
  FixedPointScalar RK_inv_12 = Rt_vec[4]*cx_inv + Rt_vec[5]*cy_inv + Rt_vec[6];
  FixedPointScalar RK_inv_20 = Rt_vec[8]*fx_inv;
  FixedPointScalar RK_inv_21 = Rt_vec[9]*fy_inv;
  FixedPointScalar RK_inv_22 = Rt_vec[8]*cx_inv + Rt_vec[9]*cy_inv + Rt_vec[10];
  
  FixedPointScalar KRK_inv_00 = fx*RK_inv_00 + cx*RK_inv_20;
  FixedPointScalar KRK_inv_01 = fx*RK_inv_01 + cx*RK_inv_21;
  FixedPointScalar KRK_inv_02 = fx*RK_inv_02 + cx*RK_inv_22;
  FixedPointScalar KRK_inv_10 = fy*RK_inv_10 + cy*RK_inv_20;
  FixedPointScalar KRK_inv_11 = fy*RK_inv_11 + cy*RK_inv_21;
  FixedPointScalar KRK_inv_12 = fy*RK_inv_12 + cy*RK_inv_22;
  FixedPointScalar KRK_inv_20 = RK_inv_20;
  FixedPointScalar KRK_inv_21 = RK_inv_21;
  FixedPointScalar KRK_inv_22 = RK_inv_22;
  FixedPointScalar Kt_0 = fx*Rt_vec[3] + cx*Rt_vec[11];
  FixedPointScalar Kt_1 = fy*Rt_vec[7] + cy*Rt_vec[11];
  FixedPointScalar Kt_2 = Rt_vec[11];
  int correspCount = 0;
  Mat corresps(rows, cols, CV_16SC2, Scalar::all(-1));
  Rect r(0, 0, cols, rows);
  for(int v1 = 0; v1 < rows; v1++)
  {
     for(int u1 = 0; u1 < cols; u1++)
     {
         if(selectMask1.at<uchar>(v1, u1))
         {
             FixedPointScalar d1 = d1_vec[v1*cols + u1];
             FixedPointScalar u1_shift ((FIXP_SCALAR_TYPE)u1, fpconfig);
             FixedPointScalar v1_shift ((FIXP_SCALAR_TYPE)v1, fpconfig);
             FixedPointScalar transformed_d1_shift = KRK_inv_20*u1_shift + KRK_inv_21*v1_shift + KRK_inv_22;
             transformed_d1_shift = (d1*transformed_d1_shift) + Kt_2;
             if(mpz_get_si(transformed_d1_shift.big_value) > 0)
             {
                 FixedPointScalar u0_shift = KRK_inv_00*u1_shift + KRK_inv_01*v1_shift + KRK_inv_02;
                 FixedPointScalar v0_shift = KRK_inv_10*u1_shift + KRK_inv_11*v1_shift + KRK_inv_12;
                 u0_shift = (d1*u0_shift) + Kt_0;
                 v0_shift = (d1*v0_shift) + Kt_1;
                 u0_shift = u0_shift / transformed_d1_shift;
                 v0_shift = v0_shift / transformed_d1_shift;
                 //int u0 = (int)round(u0_shift.value_floating);
                 //int v0 = (int)round(v0_shift.value_floating); 
                 int u0 = (int)round(u0_shift.to_floating());
                 int v0 = (int)round(v0_shift.to_floating()); 
                 //exit(1);
                 if(r.contains(Point(u0,v0)))
                 {
                     FixedPointScalar d0 = d0_vec[v0*cols + u0];
                     //if(validMask0.at<uchar>(v0, u0) && std::abs(mpz_get_si(transformed_d1_shift.big_value) - mpz_get_si(d0.big_value)) <= (maxDepthDiff*(1LL<<shift)))
                     if(validMask0.at<uchar>(v0, u0) && std::abs(transformed_d1_shift.to_floating() - d0.to_floating()) <= maxDepthDiff && std::abs(v1 - v0) <= maxLineDiff)
                     {
                            Vec2s& c = corresps.at<Vec2s>(v0,u0);
                            if(c[0] != -1)
                            {
                                int exist_u1 = c[0], exist_v1 = c[1];
                                FixedPointScalar exist_u1_shift ((FIXP_SCALAR_TYPE)exist_u1, fpconfig);
                                FixedPointScalar exist_v1_shift ((FIXP_SCALAR_TYPE)exist_v1, fpconfig);
                                FixedPointScalar exist_d1 = d1_vec[exist_v1*cols + exist_u1];
                                FixedPointScalar exist_d1_shift = KRK_inv_20*exist_u1_shift + KRK_inv_21*exist_v1_shift + KRK_inv_22;
                                exist_d1_shift = (exist_d1*exist_d1_shift) + Kt_2;
                                if(mpz_get_si(transformed_d1_shift.big_value) > mpz_get_si(exist_d1_shift.big_value))
                                    continue;
                            }
                            else
                                correspCount++;

                            c = Vec2s((short)u1, (short)v1);

                     }
                 }
             }

         }
     }
  }

  int v_max_corr = 0;
  _corresps.create(correspCount, 1, CV_32SC4);
  Vec4i * corresps_ptr = _corresps.ptr<Vec4i>();
  for(int v0 = 0, i = 0; v0 < corresps.rows; v0++)
  {
      const Vec2s* corresps_row = corresps.ptr<Vec2s>(v0);
      for(int u0 = 0; u0 < corresps.cols; u0++)
      {
          const Vec2s& c = corresps_row[u0];
          if(c[0] != -1)
            {
              //corresps_ptr[i++] = Vec4i(u0,v0,c[0],c[1]);
              corresps_ptr[i++] = Vec4i(c[0],c[1],u0,v0);
              int v_diff = abs(v0-c[1]);
              if(v_diff > v_max_corr)
                  v_max_corr = v_diff;
            }
      }
  }
  return v_max_corr;
}

typedef
void (*CalcRgbdEquationCoeffsPtr)(float*, float, float, const Point3f&, float, float);

typedef
void (*CalcICPEquationCoeffsPtr)(float*, const Point3f&, const Vec3f&);

typedef
void (*CalcFeatureXEquationCoeffsPtr)(float*, const Point3f&, float);

typedef
void (*CalcFeatureYEquationCoeffsPtr)(float*, const Point3f&, float);
/*
static
void calcRgbdLsmMatrices(const Mat& image0, const vector<FixedPointVector>& cloud0, const Mat& Rt,
               const Mat& image1, const Mat& dI_dx1, const Mat& dI_dy1,
               const Mat& corresps, float fx, float fy, float sobelScaleIn,
               vector<FixedPointScalar>& A_vec, vector<FixedPointScalar>& B_vec, CalcRgbdEquationCoeffsPtr func, int transformDim)
{
    FixedPointScalar correspsCount((FIXP_SCALAR_TYPE)corresps.rows, fpconfig);
    FixedPointScalar fx_fix((FIXP_SCALAR_TYPE)fx, fpconfig);
    FixedPointScalar fy_fix((FIXP_SCALAR_TYPE)fy, fpconfig);

    vector<FixedPointScalar> Rt_vec;
    Rt_vec = f_Mat2Vec(Rt, fpconfig); //float

    vector<FixedPointScalar> diffs_ptr;
    FixedPointScalar sigma((FIXP_SCALAR_TYPE)0, fpconfig);

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
         const Vec4i& c = corresps_ptr[correspIndex];
         int u0 = c[0], v0 = c[1];
         int u1 = c[2], v1 = c[3];

         FixedPointScalar diffs ((FIXP_SCALAR_TYPE)(static_cast<int>(image0.at<uchar>(v0,u0))-static_cast<int>(image1.at<uchar>(v1,u1))), fpconfig);
         diffs_ptr.push_back(diffs);
         sigma += diffs * diffs;
    }
    FixedPointScalar sigma_final = (sigma/correspsCount).sqrt();

    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
         const Vec4i& c = corresps_ptr[correspIndex];
         int u0 = c[0], v0 = c[1];
         int u1 = c[2], v1 = c[3];

         //double w = sigma + std::abs(diffs_ptr[correspIndex]);
         //w = w > DBL_EPSILON ? 1./w : 1.;
         FixedPointScalar w_tmp = sigma_final + diffs_ptr[correspIndex].abs();
         FixedPointScalar one_fix((FIXP_SCALAR_TYPE)1, fpconfig);
         FixedPointScalar w = one_fix;
         if(mpz_get_si(w_tmp.big_value) == 0)
         {
           w = one_fix;
         }
         else
         {
           w = one_fix / w_tmp;
         }

         FixedPointScalar sobelScaleIn_fix((FIXP_SCALAR_TYPE)sobelScaleIn, fpconfig);
         FixedPointScalar w_sobelScale = w * sobelScaleIn_fix;
         FixedPointVector p0 = cloud0[v0*image0.cols + u0];
         FixedPointScalar tp0x = p0.x * Rt_vec[0] + p0.y * Rt_vec[1] + p0.z * Rt_vec[2] + Rt_vec[3];
         FixedPointScalar tp0y = p0.x * Rt_vec[4] + p0.y * Rt_vec[5] + p0.z * Rt_vec[6] + Rt_vec[7];
         FixedPointScalar tp0z = p0.x * Rt_vec[8] + p0.y * Rt_vec[9] + p0.z * Rt_vec[10] + Rt_vec[11];

         FixedPointScalar neg_one(-1.0f, fpconfig);
         FixedPointScalar dI_dx1_fix((FIXP_SCALAR_TYPE)dI_dx1.at<short int>(v1,u1), fpconfig);
         FixedPointScalar dI_dy1_fix((FIXP_SCALAR_TYPE)dI_dy1.at<short int>(v1,u1), fpconfig);
         FixedPointScalar dIdx = w_sobelScale * dI_dx1_fix;
         FixedPointScalar dIdy = w_sobelScale * dI_dy1_fix;
         FixedPointScalar invz = one_fix / tp0z;
         FixedPointScalar v0_fix = dIdx * fx_fix * invz;
         FixedPointScalar v1_fix = dIdy * fy_fix * invz;
         FixedPointScalar v2_fix = v0_fix * tp0x + v1_fix * tp0y;
         v2_fix = neg_one * v2_fix * invz;

         FixedPointScalar zero_fix((FIXP_SCALAR_TYPE)0, fpconfig);
         vector<FixedPointScalar> C_vec(6, zero_fix);
         C_vec[0] = neg_one * tp0z * v1_fix + tp0y * v2_fix;
         C_vec[1] = tp0z * v0_fix - tp0x * v2_fix;
         C_vec[2] = neg_one * tp0y * v0_fix + tp0x * v1_fix;
         C_vec[3] = v0_fix;
         C_vec[4] = v1_fix;
         C_vec[5] = v2_fix;

         for(int y = 0; y < transformDim; y++)
         {
             for(int x = y; x < transformDim; x++)
             {
                 FixedPointScalar  test = C_vec[y] * C_vec[x];
                 A_vec[y*transformDim + x] = A_vec[y*transformDim + x] + test;
             }
             B_vec[y] = B_vec[y] + (C_vec[y] * w * diffs_ptr[correspIndex]);
         }
    }

    for(int y = 0; y < transformDim; y++)
        for(int x = y+1; x < transformDim; x++)
        {
            A_vec[x*transformDim + y] = A_vec[y*transformDim + x];
        }
}

static
void calcICPLsmMatrices(const vector<FixedPointVector>& cloud0, const Mat& Rt,
                        const vector<FixedPointVector>& cloud1, const vector<FixedPointVector>& normals1,
                        const Mat& corresps,
                        //Mat& AtA, Mat& AtB, CalcICPEquationCoeffsPtr func, int transformDim)
                        vector<FixedPointScalar>& A_vec, vector<FixedPointScalar>& B_vec, CalcICPEquationCoeffsPtr func, int transformDim, int cols)
{
    
    FixedPointScalar correspsCount((FIXP_SCALAR_TYPE)corresps.rows, fpconfig);

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    vector<FixedPointScalar> Rt_vec;
    Rt_vec = f_Mat2Vec(Rt, fpconfig); //float

    vector<FixedPointScalar> diffs_ptr;
    vector<FixedPointVector> tps0_ptr;
    FixedPointScalar sigma((FIXP_SCALAR_TYPE)0, fpconfig);
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u0 = c[0], v0 = c[1];
        int u1 = c[2], v1 = c[3];

        FixedPointVector p0 = cloud0[v0*cols + u0];
        FixedPointScalar p0x = p0.x;
        FixedPointScalar p0y = p0.y;
        FixedPointScalar p0z = p0.z;


        FixedPointScalar tp0x = p0x * Rt_vec[0] + p0y * Rt_vec[1] + p0z * Rt_vec[2] + Rt_vec[3];
        FixedPointScalar tp0y = p0x * Rt_vec[4] + p0y * Rt_vec[5] + p0z * Rt_vec[6] + Rt_vec[7];
        FixedPointScalar tp0z = p0x * Rt_vec[8] + p0y * Rt_vec[9] + p0z * Rt_vec[10] + Rt_vec[11];

        FixedPointVector n1 = normals1[v1*cols + u1];
        FixedPointScalar n1x = n1.x;
        FixedPointScalar n1y = n1.y;
        FixedPointScalar n1z = n1.z;

        FixedPointVector p1 = cloud1[v1*cols + u1];
        FixedPointScalar p1x = p1.x;
        FixedPointScalar p1y = p1.y;
        FixedPointScalar p1z = p1.z;

        FixedPointVector v (p1x - tp0x, p1y - tp0y, p1z - tp0z);

        FixedPointVector tp0(tp0x, tp0y, tp0z);
        tps0_ptr.push_back(tp0);
        FixedPointScalar diffs = n1x * v.x + n1y * v.y + n1z * v.z;
        diffs_ptr.push_back(diffs);
        sigma += diffs * diffs;
    }

    FixedPointScalar sigma_final = (sigma/correspsCount).sqrt();

    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u1 = c[2], v1 = c[3];
        
        FixedPointScalar w_tmp = sigma_final + diffs_ptr[correspIndex].abs();
        FixedPointScalar one_fix((FIXP_SCALAR_TYPE)1, fpconfig);
        FixedPointScalar w = one_fix;
        //if(w_tmp.value == 0)
        if(mpz_get_si(w_tmp.big_value) == 0)
        {
          w = one_fix;
        }
        else
        {
          w = one_fix / w_tmp;
        }
        
        FixedPointVector n1 = normals1[v1*cols + u1];
        FixedPointScalar n1x = n1.x;
        FixedPointScalar n1y = n1.y;
        FixedPointScalar n1z = n1.z;
        n1x = n1x * w;
        n1y = n1y * w;
        n1z = n1z * w;

        FixedPointVector tp0 = tps0_ptr[correspIndex];
        FixedPointScalar neg_one(-1.0f, fpconfig);
        FixedPointScalar zero_fix((FIXP_SCALAR_TYPE)0, fpconfig);
        vector<FixedPointScalar> C_vec(6, zero_fix);
        FixedPointScalar c0 = neg_one * tp0.z * n1y + tp0.y * n1z;
        FixedPointScalar c1 = tp0.z * n1x - tp0.x * n1z;
        FixedPointScalar c2 = neg_one * tp0.y * n1x + tp0.x * n1y;
        C_vec[0] = c0;
        C_vec[1] = c1;
        C_vec[2] = c2;
        C_vec[3] = n1x;
        C_vec[4] = n1y;
        C_vec[5] = n1z;
        
        for(int y = 0; y < transformDim; y++)
        {
            for(int x = y; x < transformDim; x++)
            {
                FixedPointScalar  test = C_vec[y] * C_vec[x];
                A_vec[y*transformDim + x] = A_vec[y*transformDim + x] + test;
            }
            B_vec[y] = B_vec[y] + (C_vec[y] * w * diffs_ptr[correspIndex]);
        }
    }

    for(int y = 0; y < transformDim; y++)
        for(int x = y+1; x < transformDim; x++)
        {
            A_vec[x*transformDim + y] = A_vec[y*transformDim + x];
        }
}
*/

static
void calcRgbdLsmMatrices(const Mat& image0, const Mat& cloud0, const Mat& Rt,
               const Mat& image1, const Mat& dI_dx1, const Mat& dI_dy1,
               const Mat& corresps, float fx, float fy, float sobelScaleIn,
               Mat& AtA, Mat& AtB, CalcRgbdEquationCoeffsPtr func, int transformDim)
{
    AtA = Mat(transformDim, transformDim, CV_32FC1, Scalar(0));
    AtB = Mat(transformDim, 1, CV_32FC1, Scalar(0));
    float* AtB_ptr = AtB.ptr<float>();

    const int correspsCount = corresps.rows;

    CV_Assert(Rt.type() == CV_32FC1);
    const float * Rt_ptr = Rt.ptr<const float>();

    AutoBuffer<float> diffs(correspsCount);
    float* diffs_ptr = diffs;

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    float sigma = 0;
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
         const Vec4i& c = corresps_ptr[correspIndex];
         int u0 = c[0], v0 = c[1];
         int u1 = c[2], v1 = c[3];

         diffs_ptr[correspIndex] = static_cast<float>(static_cast<int>(image0.at<uchar>(v0,u0)) -
                                                      static_cast<int>(image1.at<uchar>(v1,u1)));
         //std::cout << "====================test=======================" << diffs_ptr[0] <<  std::endl;
         //std::cout << static_cast<int>(image0.at<uchar>(v0,u0)) <<  std::endl;
         //std::cout << static_cast<int>(image1.at<uchar>(v1,u1)) <<  std::endl;
	 //exit(1);
         sigma += diffs_ptr[correspIndex] * diffs_ptr[correspIndex];
    }
    sigma = std::sqrt(sigma/correspsCount);

    std::vector<float> A_buf(transformDim);
    float* A_ptr = &A_buf[0];

    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
         const Vec4i& c = corresps_ptr[correspIndex];
         int u0 = c[0], v0 = c[1];
         int u1 = c[2], v1 = c[3];

         float w = sigma + std::abs(diffs_ptr[correspIndex]);
         w = w > DBL_EPSILON ? 1./w : 1.;

         float w_sobelScale = w * sobelScaleIn;

         const Point3f& p0 = cloud0.at<Point3f>(v0,u0);
         Point3f tp0;
         tp0.x = (float)(p0.x * Rt_ptr[0] + p0.y * Rt_ptr[1] + p0.z * Rt_ptr[2] + Rt_ptr[3]);
         tp0.y = (float)(p0.x * Rt_ptr[4] + p0.y * Rt_ptr[5] + p0.z * Rt_ptr[6] + Rt_ptr[7]);
         tp0.z = (float)(p0.x * Rt_ptr[8] + p0.y * Rt_ptr[9] + p0.z * Rt_ptr[10] + Rt_ptr[11]);

         func(A_ptr,
              w_sobelScale * dI_dx1.at<short int>(v1,u1),
              w_sobelScale * dI_dy1.at<short int>(v1,u1),
              tp0, fx, fy);

        for(int y = 0; y < transformDim; y++)
        {
            float* AtA_ptr = AtA.ptr<float>(y);
            for(int x = y; x < transformDim; x++)
                AtA_ptr[x] += A_ptr[y] * A_ptr[x];

            AtB_ptr[y] += A_ptr[y] * w * diffs_ptr[correspIndex];
        }
    }

    for(int y = 0; y < transformDim; y++)
        for(int x = y+1; x < transformDim; x++)
            AtA.at<float>(x,y) = AtA.at<float>(y,x);
}

static
void calcICPLsmMatrices(const Mat& cloud0, const Mat& Rt,
                        const Mat& cloud1, const Mat& normals1,
                        const Mat& corresps,
                        Mat& AtA, Mat& AtB, CalcICPEquationCoeffsPtr func, int transformDim)
{
    AtA = Mat(transformDim, transformDim, CV_32FC1, Scalar(0));
    AtB = Mat(transformDim, 1, CV_32FC1, Scalar(0));
    float* AtB_ptr = AtB.ptr<float>();

    const int correspsCount = corresps.rows;

    CV_Assert(Rt.type() == CV_32FC1);
    const float * Rt_ptr = Rt.ptr<const float>();

    AutoBuffer<float> diffs(correspsCount);
    float * diffs_ptr = diffs;

    AutoBuffer<Point3f> transformedPoints0(correspsCount);
    Point3f * tps0_ptr = transformedPoints0;

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    float sigma = 0;
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u0 = c[0], v0 = c[1];
        int u1 = c[2], v1 = c[3];

        const Point3f& p0 = cloud0.at<Point3f>(v0,u0);
        Point3f tp0;
        tp0.x = (float)(p0.x * Rt_ptr[0] + p0.y * Rt_ptr[1] + p0.z * Rt_ptr[2] + Rt_ptr[3]);
        tp0.y = (float)(p0.x * Rt_ptr[4] + p0.y * Rt_ptr[5] + p0.z * Rt_ptr[6] + Rt_ptr[7]);
        tp0.z = (float)(p0.x * Rt_ptr[8] + p0.y * Rt_ptr[9] + p0.z * Rt_ptr[10] + Rt_ptr[11]);

        Vec3f n1 = normals1.at<Vec3f>(v1, u1);
        Point3f v = cloud1.at<Point3f>(v1,u1) - tp0;

        tps0_ptr[correspIndex] = tp0;
        diffs_ptr[correspIndex] = n1[0] * v.x + n1[1] * v.y + n1[2] * v.z;
        //std::cout << "====================test=======================" << diffs_ptr[0] <<  std::endl;
        //exit(1);
        sigma += diffs_ptr[correspIndex] * diffs_ptr[correspIndex];
    }

    sigma = std::sqrt(sigma/correspsCount);

    std::vector<float> A_buf(transformDim);
    float* A_ptr = &A_buf[0];
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u1 = c[2], v1 = c[3];

        float w = sigma + std::abs(diffs_ptr[correspIndex]);
        w = w > DBL_EPSILON ? 1./w : 1.;

        func(A_ptr, tps0_ptr[correspIndex], normals1.at<Vec3f>(v1, u1) * w);

        for(int y = 0; y < transformDim; y++)
        {
            float* AtA_ptr = AtA.ptr<float>(y);
            for(int x = y; x < transformDim; x++)
                AtA_ptr[x] += A_ptr[y] * A_ptr[x];

            AtB_ptr[y] += A_ptr[y] * w * diffs_ptr[correspIndex];
        }
    }

    for(int y = 0; y < transformDim; y++)
        for(int x = y+1; x < transformDim; x++)
            AtA.at<float>(x,y) = AtA.at<float>(y,x);
}


void calcFeatureLsmMatrices(const Mat& cloud0, const Mat& Rt,
               const Mat& corresps, float fx, float fy, float cx, float cy,
               Mat& AtA, Mat& AtB, CalcFeatureXEquationCoeffsPtr func_x, CalcFeatureYEquationCoeffsPtr func_y, int transformDim)
{
    AtA = Mat(transformDim, transformDim, CV_32FC1, Scalar(0));
    AtB = Mat(transformDim, 1, CV_32FC1, Scalar(0));
    float* AtB_ptr = AtB.ptr<float>();

    const int correspsCount = corresps.rows;

    CV_Assert(Rt.type() == CV_32FC1);
    const float * Rt_ptr = Rt.ptr<const float>();

    AutoBuffer<float> diffs_x(correspsCount);
    AutoBuffer<float> diffs_y(correspsCount);
    float* diffs_x_ptr = diffs_x;
    float* diffs_y_ptr = diffs_y;

    AutoBuffer<Point3f> transformedPoints0(correspsCount);
    Point3f * tps0_ptr = transformedPoints0;

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    float sigma_x = 0;
    float sigma_y = 0;
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u0 = c[0], v0 = c[1];
        int u1 = c[2], v1 = c[3];
    
        const Point3f& p0 = cloud0.at<Point3f>(v0,u0);
        Point3f tp0;
        tp0.x = (float)(p0.x * Rt_ptr[0] + p0.y * Rt_ptr[1] + p0.z * Rt_ptr[2] + Rt_ptr[3]);
        tp0.y = (float)(p0.x * Rt_ptr[4] + p0.y * Rt_ptr[5] + p0.z * Rt_ptr[6] + Rt_ptr[7]);
        tp0.z = (float)(p0.x * Rt_ptr[8] + p0.y * Rt_ptr[9] + p0.z * Rt_ptr[10] + Rt_ptr[11]);
        int p2d_x = cvRound(fx * tp0.x / tp0.z + cx);
        int p2d_y = cvRound(fy * tp0.y / tp0.z + cy);

        tps0_ptr[correspIndex] = tp0;
        //diffs_x_ptr[correspIndex] = p2d_x - u1;
        //diffs_y_ptr[correspIndex] = p2d_y - v1;
        diffs_x_ptr[correspIndex] = u1 - p2d_x;
        diffs_y_ptr[correspIndex] = v1 - p2d_y;
        sigma_x += diffs_x_ptr[correspIndex] * diffs_x_ptr[correspIndex];
        sigma_y += diffs_y_ptr[correspIndex] * diffs_y_ptr[correspIndex];
    }
    //exit(1);

    sigma_x = std::sqrt(sigma_x/correspsCount);
    sigma_y = std::sqrt(sigma_y/correspsCount);

    std::vector<float> A_buf_x(transformDim);
    std::vector<float> A_buf_y(transformDim);
    float* A_ptr_x = &A_buf_x[0];
    float* A_ptr_y = &A_buf_y[0];
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        float w_x = sigma_x + std::abs(diffs_x_ptr[correspIndex]);
        float w_y = sigma_y + std::abs(diffs_y_ptr[correspIndex]);
        w_x = w_x > DBL_EPSILON ? 1./w_x : 1.;
        w_y = w_y > DBL_EPSILON ? 1./w_y : 1.;

        func_x(A_ptr_x, tps0_ptr[correspIndex], fx * w_x);
        func_y(A_ptr_y, tps0_ptr[correspIndex], fy * w_y);

        for(int y = 0; y < transformDim; y++)
        {
            float* AtA_ptr = AtA.ptr<float>(y);
            for(int x = y; x < transformDim; x++)
                AtA_ptr[x] += A_ptr_x[y] * A_ptr_x[x] + A_ptr_y[y] * A_ptr_y[x];

            AtB_ptr[y] += A_ptr_x[y] * w_x * diffs_x_ptr[correspIndex] + A_ptr_y[y] * w_y * diffs_y_ptr[correspIndex];
        }
    }

    for(int y = 0; y < transformDim; y++)
        for(int x = y+1; x < transformDim; x++)
            AtA.at<float>(x,y) = AtA.at<float>(y,x);

}

static
bool solveSystem(vector<FixedPointScalar>& A_vec, vector<FixedPointScalar>& B_vec, double detThreshold, Mat& x)
{
    FixedPointScalar zero_fix((FIXP_SCALAR_TYPE)0, fpconfig);
    vector<FixedPointScalar> A_vec2(6*6, zero_fix);
    vector<FixedPointScalar> B_vec2(6, zero_fix);

    int rows = 6;
    int cols = 6;
    if(mpz_get_si(A_vec[0].big_value)==0)
    {
        cout << "===========DIV 0===================== " << endl;
        return false;
    }
    A_vec2[0] = A_vec[0];
    A_vec2[1] = A_vec[1];
    A_vec2[2] = A_vec[2];
    A_vec2[3] = A_vec[3];
    A_vec2[4] = A_vec[4];
    A_vec2[5] = A_vec[5];
    A_vec2[6] = A_vec[1]/A_vec[0];
    A_vec2[12] = A_vec[2]/A_vec[0];
    A_vec2[18] = A_vec[3]/A_vec[0];
    A_vec2[24] = A_vec[4]/A_vec[0];
    A_vec2[30] = A_vec[5]/A_vec[0];
    for(int k = 0; k < rows; k++)
    {
        for(int m = 0; m < k; m++)
        {   
            if(m==0)
            {
                A_vec2[k*cols + k] = A_vec[k*cols + k] - (A_vec2[m*cols + k] * A_vec2[k*cols + m]);
 
            }
            else
                A_vec2[k*cols + k] = A_vec2[k*cols + k] - (A_vec2[m*cols + k] * A_vec2[k*cols + m]);
        }
        
        for(int i = k+1; i < cols; i++)
        {
            for(int m = 0; m < k; m++)
            {
                if(m==0)
                    A_vec2[k*cols + i] = A_vec[k*cols + i] - (A_vec2[m*cols + i] * A_vec2[k*cols + m]);
                else
                    A_vec2[k*cols + i] = A_vec2[k*cols + i] - (A_vec2[m*cols + i] * A_vec2[k*cols + m]);
            }
            //if(A_vec2[k*cols + k].value==0)
            if(mpz_get_si(A_vec2[k*cols + k].big_value)==0)
            {
                cout << "===========DIV 1===================== " << endl;
                return false;
            }
          
            A_vec2[i*cols + k] = A_vec2[k*cols + i] / A_vec2[k*cols + k] ;
        }

    }

    B_vec2[0] = B_vec[0];
    for(int i = 0; i < rows; i++)
    {
        for(int k = 0; k < i; k++)
        {
            if(k==0)
                B_vec2[i] = B_vec[i] - (A_vec2[i*cols + k]*B_vec2[k]) ;
            else
                B_vec2[i] = B_vec2[i] - (A_vec2[i*cols + k]*B_vec2[k]) ;
        }
    }

    for(int i = rows-1; i >= 0; i--)
    {
        //if(A_vec2[i*cols + i].value==0)
        if(mpz_get_si(A_vec2[i*cols + i].big_value)==0)
            {
            cout << "===========DIV 2===================== " << endl;
                return false;
            }
        B_vec2[i] = B_vec2[i] / A_vec2[i*cols + i];
        for(int k = i+1; k < rows; k++)
        {
            B_vec2[i] = B_vec2[i] - (A_vec2[k*cols + i]*B_vec2[k]) ;
        }
    }


    x = Vec2Mat_f(B_vec2, 6, 1);
    return true;
}

static
bool solveSystem_ori(const Mat& AtA, const Mat& AtB, float detThreshold, Mat& x)
{
    float det = determinant(AtA);

    if(fabs (det) < detThreshold || cvIsNaN(det) || cvIsInf(det))
        return false;

    solve(AtA, AtB, x, DECOMP_CHOLESKY);

    return true;
}

static
bool testDeltaTransformation(const Mat& deltaRt, float maxTranslation, float maxRotation)
{
    float translation = norm(deltaRt(Rect(3, 0, 1, 3)));

    Mat rvec;
    Rodrigues(deltaRt(Rect(0,0,3,3)), rvec);

    float rotation = norm(rvec) * 180. / CV_PI;

    return translation <= maxTranslation && rotation <= maxRotation;
}

static
void computeProjectiveMatrix(const Mat& ksi, Mat& Rt)
{
    CV_Assert(ksi.size() == Size(1,6) && ksi.type() == CV_32FC1);

#ifdef HAVE_EIGEN3_HERE
    const float* ksi_ptr = ksi.ptr<const float>();
    Eigen::Matrix<float,4,4> twist, g;
    twist << 0.,          -ksi_ptr[2], ksi_ptr[1],  ksi_ptr[3],
             ksi_ptr[2],  0.,          -ksi_ptr[0], ksi_ptr[4],
             -ksi_ptr[1], ksi_ptr[0],  0,           ksi_ptr[5],
             0.,          0.,          0.,          0.;
    g = twist.exp();

    eigen2cv(g, Rt);
#else
    // TODO: check computeProjectiveMatrix when there is not eigen library,
    //       because it gives less accurate pose of the camera
    Rt = Mat::eye(4, 4, CV_32FC1);

    Mat R = Rt(Rect(0,0,3,3));
    Mat rvec = ksi.rowRange(0,3);

    Rodrigues(rvec, R);

    Rt.at<float>(0,3) = ksi.at<float>(3);
    Rt.at<float>(1,3) = ksi.at<float>(4);
    Rt.at<float>(2,3) = ksi.at<float>(5);
#endif
}

static inline
void calcRgbdEquationCoeffs(float* C, float dIdx, float dIdy, const Point3f& p3d, float fx, float fy)
{
    float invz  = 1. / p3d.z,
           v0 = dIdx * fx * invz,
           v1 = dIdy * fy * invz,
           v2 = -(v0 * p3d.x + v1 * p3d.y) * invz;

    C[0] = -p3d.z * v1 + p3d.y * v2;
    C[1] =  p3d.z * v0 - p3d.x * v2;
    C[2] = -p3d.y * v0 + p3d.x * v1;
    C[3] = v0;
    C[4] = v1;
    C[5] = v2;
}

static inline
void calcICPEquationCoeffs(float* C, const Point3f& p0, const Vec3f& n1)
{
    C[0] = -p0.z * n1[1] + p0.y * n1[2];
    C[1] =  p0.z * n1[0] - p0.x * n1[2];
    C[2] = -p0.y * n1[0] + p0.x * n1[1];
    C[3] = n1[0];
    C[4] = n1[1];
    C[5] = n1[2];
}

static inline
void calcFeatureXEquationCoeffs(float* C, const Point3f& p3d, float fx)
{
    float invz  = 1. / p3d.z;

    C[0] = -(fx * p3d.x * p3d.y * invz * invz);
    C[1] = fx + fx * p3d.x * p3d.x * invz * invz;
    C[2] = -(fx * p3d.y * invz);
    C[3] = fx * invz;
    C[4] = 0;
    C[5] = -(fx * p3d.x * invz * invz);
}

static inline
void calcFeatureYEquationCoeffs(float* C, const Point3f& p3d, float fy)
{
    float invz  = 1. / p3d.z;

    C[0] = -fy - (fy * p3d.y * p3d.y * invz * invz);
    C[1] = fy * p3d.x * p3d.x * invz * invz;
    C[2] = fy * p3d.x * invz;
    C[3] = 0;
    C[4] = fy * invz;
    C[5] = -(fy * p3d.y * invz * invz);
}

bool Odometry::compute(Ptr<OdometryFrame>& srcFrame, Ptr<OdometryFrame>& dstFrame, Mat& Rt, int& v_max, const Mat& initRt) const
{
    Size srcSize = prepareFrameCache(srcFrame, OdometryFrame::CACHE_SRC);
    Size dstSize = prepareFrameCache(dstFrame, OdometryFrame::CACHE_DST);

    if(srcSize != dstSize)
        CV_Error(Error::StsBadSize, "srcFrame and dstFrame have to have the same size (resolution).");

    int transformDim = 6;
    CalcRgbdEquationCoeffsPtr rgbdEquationFuncPtr = calcRgbdEquationCoeffs;
    CalcICPEquationCoeffsPtr icpEquationFuncPtr = calcICPEquationCoeffs;
    CalcFeatureXEquationCoeffsPtr featureXEquationFuncPtr = calcFeatureXEquationCoeffs;
    CalcFeatureYEquationCoeffsPtr featureYEquationFuncPtr = calcFeatureYEquationCoeffs;

    std::vector<int> iterCounts_vec = iterCounts;

    const int minOverdetermScale = 20;
    const int minCorrespsCount = minOverdetermScale * transformDim;

    Mat resultRt = initRt.empty() ? Mat::eye(4,4,CV_32FC1) : initRt.clone();
    Mat currRt, ksi;

    bool isOk = false;
    //for(int level = (int)iterCounts_vec.size() - 1; level >= 0; level--)
    for(int level = 0; level == 0; level++)
    {
        const Mat& levelCameraMatrix = cameraMatrix;
        const Mat& levelCameraMatrix_inv = levelCameraMatrix.inv(DECOMP_SVD);
        //const Mat& srcLevelDepth = srcFrame->depth;
        //const Mat& dstLevelDepth = dstFrame->depth;
        const vector<FixedPointScalar>& srcLevelDepth = srcFrame->depth_vec;
        const vector<FixedPointScalar>& dstLevelDepth = dstFrame->depth_vec;

        const float fx = levelCameraMatrix.at<float>(0,0);
        const float fy = levelCameraMatrix.at<float>(1,1);
        const float cx = levelCameraMatrix.at<float>(0,2);
        const float cy = levelCameraMatrix.at<float>(1,2);
        const float determinantThreshold = 1e-6;

        Mat AtA_rgbd, AtB_rgbd, AtA_icp, AtB_icp;
        Mat corresps_rgbd, corresps_icp;

        // Run transformation search on current level iteratively.
        for(int iter = 0; iter < iterCounts_vec[level]; iter ++)
        {
            Mat AtA(transformDim, transformDim, CV_32FC1, Scalar(0)), AtB(transformDim, 1, CV_32FC1, Scalar(0));
            FixedPointScalar zero_fix((int64_t)0, fpconfig);
            vector<FixedPointScalar> A_vec(transformDim*transformDim, zero_fix);
            vector<FixedPointScalar> B_vec(transformDim, zero_fix);
            vector<FixedPointScalar> A_icp_vec(transformDim*transformDim, zero_fix);
            vector<FixedPointScalar> B_icp_vec(transformDim, zero_fix);
            vector<FixedPointScalar> A_rgbd_vec(transformDim*transformDim, zero_fix);
            vector<FixedPointScalar> B_rgbd_vec(transformDim, zero_fix);
            if(iter>=5){
                Mat resultRt_inv = resultRt.inv(DECOMP_SVD);

                //int v_rgbd = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, resultRt_inv,
                //                             srcLevelDepth, srcFrame->maskDepth, dstLevelDepth, dstFrame->maskText,
                //                             maxDepthDiff, corresps_rgbd);
                int v_rgbd = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, resultRt,
                                             dstLevelDepth, dstFrame->maskDepth, srcLevelDepth, srcFrame->maskText,
                                             maxDepthDiff, srcFrame->image.rows, srcFrame->image.cols, corresps_rgbd);
                //int v_rgbd = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, resultRt_inv,
                //                             srcLevelDepth, srcFrame->maskDepth, dstLevelDepth, dstFrame->maskText,
                //                             maxDepthDiff, srcFrame->image.rows, srcFrame->image.cols, corresps_rgbd);
                if (v_rgbd > v_max)
                    v_max = v_rgbd;
                //cout << corresps_rgbd << endl;
                //cout << "v_rgbd" << v_rgbd << endl;
                //exit(1);
                //int v_icp = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, resultRt_inv,
                //                            srcLevelDepth, srcFrame->maskDepth, dstLevelDepth, dstFrame->maskNormal,
                //                            maxDepthDiff, corresps_icp);
                int v_icp = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, resultRt,
                                            dstLevelDepth, dstFrame->maskDepth, srcLevelDepth, srcFrame->maskNormal,
                                            maxDepthDiff, srcFrame->image.rows, srcFrame->image.cols, corresps_icp);
                //int v_icp = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, resultRt_inv,
                //                            srcLevelDepth, srcFrame->maskDepth, dstLevelDepth, dstFrame->maskNormal,
                //                            maxDepthDiff, srcFrame->image.rows, srcFrame->image.cols, corresps_icp);
                if (v_icp > v_max)
                    v_max = v_icp;
                if(corresps_rgbd.rows >= minCorrespsCount)
                {
                    calcRgbdLsmMatrices(srcFrame->image, srcFrame->cloud, resultRt,
                                        dstFrame->image, dstFrame->dI_dx, dstFrame->dI_dy,
                                        corresps_rgbd, fx, fy, sobelScale,
                                        AtA_rgbd, AtB_rgbd, rgbdEquationFuncPtr, transformDim);
                                        //A_rgbd_vec, B_rgbd_vec, rgbdEquationFuncPtr, transformDim);

                    AtA += AtA_rgbd;
                    AtB += AtB_rgbd;
                    //for(int i = 0; i < A_vec.size(); i ++)
                    //    A_vec[i] += A_rgbd_vec[i]; 
                    //for(int i = 0; i < B_vec.size(); i ++)
                    //    B_vec[i] += B_rgbd_vec[i]; 
                }

                if(corresps_icp.rows >= minCorrespsCount)
                {
                    calcICPLsmMatrices(srcFrame->cloud, resultRt,
                                       dstFrame->cloud, dstFrame->normals,
                                       corresps_icp, AtA_icp, AtB_icp, icpEquationFuncPtr, transformDim);
                                       //corresps_icp, A_icp_vec, B_icp_vec, icpEquationFuncPtr, transformDim, srcFrame->image.cols);
                    AtA += AtA_icp;
                    AtB += AtB_icp;
                    //for(int i = 0; i < A_vec.size(); i ++)
                    //    A_vec[i] += A_icp_vec[i]; 
                    //for(int i = 0; i < B_vec.size(); i ++)
                    //    B_vec[i] += B_icp_vec[i]; 
                }
                //AtA = Vec2Mat_f(A_vec, 6, 6);
                //AtB = Vec2Mat_f(B_vec, 6, 1);
                //Mat ksi_t;
                //bool solutionExist_t = solveSystem(A_vec, B_vec, determinantThreshold, ksi_t);
                //bool solutionExist = solveSystem_ori(AtA, AtB, determinantThreshold, ksi);
                //cout << ksi << endl;
                //cout << ksi_t << endl;
                //exit(1);
                //if(!solutionExist)
                //    break;
            }
            else 
            {
               ///////////////////////////////////////////////
               std::vector<KeyPoint> keypoints_1, keypoints_2;
               Mat descriptors_1, descriptors_2;
               Ptr<FeatureDetector> detector = ORB::create();
               //Ptr<FeatureDetector> detector = FastFeatureDetector::create();
               Ptr<DescriptorExtractor> descriptor = ORB::create();            

               detector->detect ( srcFrame->image ,keypoints_1 );
               detector->detect ( dstFrame->image ,keypoints_2 );
               descriptor->compute ( srcFrame->image, keypoints_1, descriptors_1 );
               descriptor->compute ( dstFrame->image, keypoints_2, descriptors_2 );

               Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
               vector<DMatch> matches;
               matcher->match ( descriptors_1, descriptors_2, matches );

               float min_dist=10000, max_dist=0;
               for ( int i = 0; i < descriptors_1.rows; i++ )
               {
                   float dist = matches[i].distance;
                   if ( dist < min_dist ) min_dist = dist;
                   if ( dist > max_dist ) max_dist = dist;
               }

               std::vector< DMatch > good_matches;
               for ( int i = 0; i < descriptors_1.rows; i++ )
               {
                   if ( matches[i].distance <= max ( 2*min_dist, 30.0f ) )
                   {
                       if(srcFrame->maskDepth.at<uchar>(keypoints_1[matches[i].queryIdx].pt.y, keypoints_1[matches[i].queryIdx].pt.x))
                           good_matches.push_back ( matches[i] );
                   }
               }

               //BFMatcher matcher;
               //std::vector<vector< DMatch >> matches;
               //matcher.knnMatch(descriptors_1, descriptors_2, matches, 2);
               //
               //std::vector< DMatch > good_matches;
               //for (int i = 0; i < matches.size(); i ++) {
               //    float rejectRatio = 0.8;
               //    if (matches[i][0].distance / matches[i][1].distance > rejectRatio)
               //        continue;
               //    if(srcFrame->pyramidMask[level].at<uchar>(keypoints_1[matches[i][0].queryIdx].pt.y, keypoints_1[matches[i][0].queryIdx].pt.x))
               //        good_matches.push_back(matches[i][0]);
               //}

               Mat corresps_feature;
               corresps_feature.create(good_matches.size(), 1, CV_32SC4);
               Vec4i * corresps_feature_ptr = corresps_feature.ptr<Vec4i>();
               for(int idx = 0, i = 0; idx < good_matches.size(); idx++)
               {
                   corresps_feature_ptr[i++] = Vec4i(keypoints_1[good_matches[idx].queryIdx].pt.x, keypoints_1[good_matches[idx].queryIdx].pt.y, 
                                                     keypoints_2[good_matches[idx].trainIdx].pt.x, keypoints_2[good_matches[idx].trainIdx].pt.y);
               }
               //cout << "corresps " << corresps_feature << endl;
               //cout << "corresps " << corresps_feature.size() << endl;
               //exit(1);

               Mat AtA_feature, AtB_feature;
               calcFeatureLsmMatrices(srcFrame->cloud, resultRt,
                                     corresps_feature, fx, fy, cx, cy,
                                     AtA_feature, AtB_feature, featureXEquationFuncPtr, featureYEquationFuncPtr, transformDim);
               AtA += AtA_feature;
               AtB += AtB_feature;
               //vector<FixedPointScalar>  A_feature_vec = f_Mat2Vec(AtA_feature, fpconfig);
               //vector<FixedPointScalar>  B_feature_vec = f_Mat2Vec(AtB_feature, fpconfig);
               //for(int i = 0; i < A_vec.size(); i ++)
               //    A_vec[i] += A_feature_vec[i]; 
               //for(int i = 0; i < B_vec.size(); i ++)
               //    B_vec[i] += B_feature_vec[i]; 
               /////bool solutionExist = solveSystem_ori(AtA, AtB, determinantThreshold, ksi);
               /////if(!solutionExist)
               /////    break;
               //cout << "AtA " << AtA << endl;
               //cout << "AtB " << AtB << endl;
               //cout << "cloud " << srcFrame->cloud << endl;
               //cout << "fx " << fx << endl;
               //cout << "iter " << iter << endl;
               //exit(1);
            }

            bool solutionExist = solveSystem_ori(AtA, AtB, determinantThreshold, ksi);
            if(!solutionExist)
                break;

            computeProjectiveMatrix(ksi, currRt);
            resultRt = currRt * resultRt;
            isOk = true;
        }
    }

    //cout << "v_max" << v_max << endl;
    Rt = resultRt;

    if(isOk)
    {
        Mat deltaRt;
        if(initRt.empty())
            deltaRt = resultRt;
        else
            deltaRt = resultRt * initRt.inv(DECOMP_SVD);

        isOk = testDeltaTransformation(deltaRt, maxTranslation, maxRotation);
    }

    return isOk;
}


