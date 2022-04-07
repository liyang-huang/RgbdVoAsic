#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "vo.hpp"

using namespace cv;
using namespace std;
//
const int maxLineDiff = 20;
const int sobelSize = 3;
const double sobelScale = 1./8.;

const bool pyramid_on = false;
const int feature_iter_num = 5;

static inline
void setDefaultIterCounts(Mat& iterCounts)
{
    if(pyramid_on)
        //iterCounts = Mat(Vec4i(7,7,7,10));
        //iterCounts = Mat(Vec4i(7,7,7,7));
        iterCounts = Mat(Vec4i(7,7,7,7));
    else
        iterCounts = Mat(Vec4i(7));
}

static inline
void setDefaultMinGradientMagnitudes(Mat& minGradientMagnitudes)
{
    if(pyramid_on)
        minGradientMagnitudes = Mat(Vec4f(10,10,10,10));
    else
        minGradientMagnitudes = Mat(Vec4f(10));
}

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
    maskDepth.release();
    maskText.release();
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
    setDefaultMinGradientMagnitudes(minGradientMagnitudes);
}

Odometry::Odometry(const Mat& _cameraMatrix,
                   float _minDepth, float _maxDepth, float _maxDepthDiff,
                   const std::vector<int>& _iterCounts,
                   const std::vector<float>& _minGradientMagnitudes,
                   float _maxPointsPart) :
                   minDepth(_minDepth), maxDepth(_maxDepth), maxDepthDiff(_maxDepthDiff),
                   iterCounts(Mat(_iterCounts).clone()),
                   minGradientMagnitudes(Mat(_minGradientMagnitudes).clone()),
                   maxPointsPart(_maxPointsPart),
                   cameraMatrix(_cameraMatrix),
                   maxTranslation(DEFAULT_MAX_TRANSLATION()), maxRotation(DEFAULT_MAX_ROTATION())
{
    if(iterCounts.empty() || minGradientMagnitudes.empty())
    {
        setDefaultIterCounts(iterCounts);
        setDefaultMinGradientMagnitudes(minGradientMagnitudes);
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

template<>
inline void
rescaleDepthTemplated<double>(const Mat& in, Mat& out)
{
  rescaleDepth(in, CV_64F, out);
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
  if (z_mat.depth() == in_depth.depth())
    z_mat = in_depth;
  else
    rescaleDepthTemplated<T>(in_depth, z_mat);

  //cout << "liyang test" << in_depth << endl;
  //exit(1);
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
  if (K_new.depth() == CV_64F)
    depthTo3dNoMask<double>(depth, K_new, points3d);
  else
    depthTo3dNoMask<float>(depth, K_new, points3d);
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

        if(!Normal.empty())
        {
            Mat validNormalMask = Normal == Normal; // otherwise it's Nan

            CV_Assert(validNormalMask.type() == CV_8UC3);

            std::vector<Mat> channelMasks;
            split(validNormalMask, channelMasks);
            validNormalMask = channelMasks[0] & channelMasks[1] & channelMasks[2];

            maskDepth &= validNormalMask;
        }
    }
}

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

        randomSubsetOfMask(texturedMask, (float)maxPointsPart);
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
        for(int y = 0; y < maskNormal.rows; y++)
        {
            const Vec3f *normals_row = normals.ptr<Vec3f>(y);
            uchar *normalsMask_row = maskNormal.ptr<uchar>(y);
            for(int x = 0; x < maskNormal.cols; x++)
            {
                Vec3f n = normals_row[x];
                if(cvIsNaN(n[0]))
                {
                    CV_DbgAssert(cvIsNaN(n[1]) && cvIsNaN(n[2]));
                    normalsMask_row[x] = 0;
                }
            }
        }
        randomSubsetOfMask(maskNormal, (float)maxPointsPart);
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


Size Odometry::prepareFrameCache(Ptr<OdometryFrame>& frame, int cacheType) const
{
    
    checkImage(frame->image);

    checkDepth(frame->depth, frame->image.size());
    
    checkMask(frame->mask, frame->image.size());

    depthTo3d(frame->depth, cameraMatrix, frame->cloud);

    //if(cacheType & OdometryFrame::CACHE_DST)
    //{
    //   if(frame->normals.empty())
    //   {
        normalsComputer(frame->cloud, frame->depth.rows, frame->depth.cols, frame->normals);
    //    }
        checkNormals(frame->normals, frame->depth.size());

        MaskGen(frame->mask, frame->depth, (float)minDepth, (float)maxDepth,
                frame->normals, frame->maskDepth);

        Sobel(frame->image, frame->dI_dx, CV_16S, 1, 0, sobelSize);
        Sobel(frame->image, frame->dI_dy, CV_16S, 0, 1, sobelSize);
        std::vector<int> minGradientMagnitudes_vec = minGradientMagnitudes;
        TexturedMaskGen(frame->dI_dx, frame->dI_dy,
                        minGradientMagnitudes_vec[0], frame->maskDepth,
                        maxPointsPart, frame->maskText);

        NormalsMaskGen(frame->normals, frame->maskDepth, maxPointsPart, frame->maskNormal);
    //}
    //else
    //{
    //    MaskGen(frame->mask, frame->depth, (float)minDepth, (float)maxDepth,
    //            frame->normals, frame->maskDepth);
    //}
    return frame->image.size();
}

static
int computeCorresps(const Mat& K, const Mat& K_inv, const Mat& Rt,
                     const Mat& depth0, const Mat& validMask0,
                     const Mat& depth1, const Mat& selectMask1, float maxDepthDiff,
                     Mat& _corresps)
{
    CV_Assert(K.type() == CV_64FC1);
    CV_Assert(K_inv.type() == CV_64FC1);
    CV_Assert(Rt.type() == CV_64FC1);

    Mat corresps(depth1.size(), CV_16SC2, Scalar::all(-1));

    Rect r(0, 0, depth1.cols, depth1.rows);
    Mat Kt = Rt(Rect(3,0,1,3)).clone();
    Kt = K * Kt;
    const double * Kt_ptr = Kt.ptr<const double>();

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
        const double * KRK_inv_ptr = KRK_inv.ptr<const double>();
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            KRK_inv0_u1[u1] = (float)(KRK_inv_ptr[0] * u1);
            KRK_inv3_u1[u1] = (float)(KRK_inv_ptr[3] * u1);
            KRK_inv6_u1[u1] = (float)(KRK_inv_ptr[6] * u1);
        }

        for(int v1 = 0; v1 < depth1.rows; v1++)
        {
            KRK_inv1_v1_plus_KRK_inv2[v1] = (float)(KRK_inv_ptr[1] * v1 + KRK_inv_ptr[2]);
            KRK_inv4_v1_plus_KRK_inv5[v1] = (float)(KRK_inv_ptr[4] * v1 + KRK_inv_ptr[5]);
            KRK_inv7_v1_plus_KRK_inv8[v1] = (float)(KRK_inv_ptr[7] * v1 + KRK_inv_ptr[8]);
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
void (*CalcRgbdEquationCoeffsPtr)(double*, double, double, const Point3f&, double, double);

typedef
void (*CalcICPEquationCoeffsPtr)(double*, const Point3f&, const Vec3f&);

typedef
void (*CalcFeatureXEquationCoeffsPtr)(double*, const Point3f&, double);

typedef
void (*CalcFeatureYEquationCoeffsPtr)(double*, const Point3f&, double);


static
void calcRgbdLsmMatrices(const Mat& image0, const Mat& cloud0, const Mat& Rt,
               const Mat& image1, const Mat& dI_dx1, const Mat& dI_dy1,
               //const Mat& corresps, double fx, double fy, double sobelScaleIn,
               const Mat& corresps, float fx, float fy, double sobelScaleIn,
               Mat& AtA, Mat& AtB, CalcRgbdEquationCoeffsPtr func, int transformDim)
{
    AtA = Mat(transformDim, transformDim, CV_64FC1, Scalar(0));
    AtB = Mat(transformDim, 1, CV_64FC1, Scalar(0));
    double* AtB_ptr = AtB.ptr<double>();

    const int correspsCount = corresps.rows;

    CV_Assert(Rt.type() == CV_64FC1);
    const double * Rt_ptr = Rt.ptr<const double>();

    AutoBuffer<float> diffs(correspsCount);
    float* diffs_ptr = diffs;

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    double sigma = 0;
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

    std::vector<double> A_buf(transformDim);
    double* A_ptr = &A_buf[0];

    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
         const Vec4i& c = corresps_ptr[correspIndex];
         int u0 = c[0], v0 = c[1];
         int u1 = c[2], v1 = c[3];

         double w = sigma + std::abs(diffs_ptr[correspIndex]);
         w = w > DBL_EPSILON ? 1./w : 1.;

         double w_sobelScale = w * sobelScaleIn;

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
            double* AtA_ptr = AtA.ptr<double>(y);
            for(int x = y; x < transformDim; x++)
                AtA_ptr[x] += A_ptr[y] * A_ptr[x];

            AtB_ptr[y] += A_ptr[y] * w * diffs_ptr[correspIndex];
        }
    }

    for(int y = 0; y < transformDim; y++)
        for(int x = y+1; x < transformDim; x++)
            AtA.at<double>(x,y) = AtA.at<double>(y,x);
}

static
void calcICPLsmMatrices(const Mat& cloud0, const Mat& Rt,
                        const Mat& cloud1, const Mat& normals1,
                        const Mat& corresps,
                        Mat& AtA, Mat& AtB, CalcICPEquationCoeffsPtr func, int transformDim)
{
    AtA = Mat(transformDim, transformDim, CV_64FC1, Scalar(0));
    AtB = Mat(transformDim, 1, CV_64FC1, Scalar(0));
    double* AtB_ptr = AtB.ptr<double>();

    const int correspsCount = corresps.rows;

    CV_Assert(Rt.type() == CV_64FC1);
    const double * Rt_ptr = Rt.ptr<const double>();

    AutoBuffer<float> diffs(correspsCount);
    float * diffs_ptr = diffs;

    AutoBuffer<Point3f> transformedPoints0(correspsCount);
    Point3f * tps0_ptr = transformedPoints0;

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    double sigma = 0;
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

    std::vector<double> A_buf(transformDim);
    double* A_ptr = &A_buf[0];
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        const Vec4i& c = corresps_ptr[correspIndex];
        int u1 = c[2], v1 = c[3];

        double w = sigma + std::abs(diffs_ptr[correspIndex]);
        w = w > DBL_EPSILON ? 1./w : 1.;

        func(A_ptr, tps0_ptr[correspIndex], normals1.at<Vec3f>(v1, u1) * w);

        for(int y = 0; y < transformDim; y++)
        {
            double* AtA_ptr = AtA.ptr<double>(y);
            for(int x = y; x < transformDim; x++)
                AtA_ptr[x] += A_ptr[y] * A_ptr[x];

            AtB_ptr[y] += A_ptr[y] * w * diffs_ptr[correspIndex];
        }
    }

    for(int y = 0; y < transformDim; y++)
        for(int x = y+1; x < transformDim; x++)
            AtA.at<double>(x,y) = AtA.at<double>(y,x);
}


void calcFeatureLsmMatrices(const Mat& cloud0, const Mat& Rt,
               const Mat& corresps, double fx, double fy, double cx, double cy,
               Mat& AtA, Mat& AtB, CalcFeatureXEquationCoeffsPtr func_x, CalcFeatureYEquationCoeffsPtr func_y, int transformDim)
{
    AtA = Mat(transformDim, transformDim, CV_64FC1, Scalar(0));
    AtB = Mat(transformDim, 1, CV_64FC1, Scalar(0));
    double* AtB_ptr = AtB.ptr<double>();

    const int correspsCount = corresps.rows;

    CV_Assert(Rt.type() == CV_64FC1);
    const double * Rt_ptr = Rt.ptr<const double>();

    AutoBuffer<float> diffs_x(correspsCount);
    AutoBuffer<float> diffs_y(correspsCount);
    float* diffs_x_ptr = diffs_x;
    float* diffs_y_ptr = diffs_y;

    AutoBuffer<Point3f> transformedPoints0(correspsCount);
    Point3f * tps0_ptr = transformedPoints0;

    const Vec4i* corresps_ptr = corresps.ptr<Vec4i>();

    double sigma_x = 0;
    double sigma_y = 0;
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

    std::vector<double> A_buf_x(transformDim);
    std::vector<double> A_buf_y(transformDim);
    double* A_ptr_x = &A_buf_x[0];
    double* A_ptr_y = &A_buf_y[0];
    for(int correspIndex = 0; correspIndex < corresps.rows; correspIndex++)
    {
        double w_x = sigma_x + std::abs(diffs_x_ptr[correspIndex]);
        double w_y = sigma_y + std::abs(diffs_y_ptr[correspIndex]);
        w_x = w_x > DBL_EPSILON ? 1./w_x : 1.;
        w_y = w_y > DBL_EPSILON ? 1./w_y : 1.;

        func_x(A_ptr_x, tps0_ptr[correspIndex], fx * w_x);
        func_y(A_ptr_y, tps0_ptr[correspIndex], fy * w_y);

        for(int y = 0; y < transformDim; y++)
        {
            double* AtA_ptr = AtA.ptr<double>(y);
            for(int x = y; x < transformDim; x++)
                AtA_ptr[x] += A_ptr_x[y] * A_ptr_x[x] + A_ptr_y[y] * A_ptr_y[x];

            AtB_ptr[y] += A_ptr_x[y] * w_x * diffs_x_ptr[correspIndex] + A_ptr_y[y] * w_y * diffs_y_ptr[correspIndex];
        }
    }

    for(int y = 0; y < transformDim; y++)
        for(int x = y+1; x < transformDim; x++)
            AtA.at<double>(x,y) = AtA.at<double>(y,x);

}

static
bool solveSystem(const Mat& AtA, const Mat& AtB, double detThreshold, Mat& x)
{
    double det = determinant(AtA);

    if(fabs (det) < detThreshold || cvIsNaN(det) || cvIsInf(det))
        return false;

    solve(AtA, AtB, x, DECOMP_CHOLESKY);

    return true;
}

static
bool testDeltaTransformation(const Mat& deltaRt, double maxTranslation, double maxRotation)
{
    double translation = norm(deltaRt(Rect(3, 0, 1, 3)));

    Mat rvec;
    Rodrigues(deltaRt(Rect(0,0,3,3)), rvec);

    double rotation = norm(rvec) * 180. / CV_PI;

    return translation <= maxTranslation && rotation <= maxRotation;
}

static
void computeProjectiveMatrix(const Mat& ksi, Mat& Rt)
{
    CV_Assert(ksi.size() == Size(1,6) && ksi.type() == CV_64FC1);

#ifdef HAVE_EIGEN3_HERE
    const double* ksi_ptr = ksi.ptr<const double>();
    Eigen::Matrix<double,4,4> twist, g;
    twist << 0.,          -ksi_ptr[2], ksi_ptr[1],  ksi_ptr[3],
             ksi_ptr[2],  0.,          -ksi_ptr[0], ksi_ptr[4],
             -ksi_ptr[1], ksi_ptr[0],  0,           ksi_ptr[5],
             0.,          0.,          0.,          0.;
    g = twist.exp();

    eigen2cv(g, Rt);
#else
    // TODO: check computeProjectiveMatrix when there is not eigen library,
    //       because it gives less accurate pose of the camera
    Rt = Mat::eye(4, 4, CV_64FC1);

    Mat R = Rt(Rect(0,0,3,3));
    Mat rvec = ksi.rowRange(0,3);

    Rodrigues(rvec, R);

    Rt.at<double>(0,3) = ksi.at<double>(3);
    Rt.at<double>(1,3) = ksi.at<double>(4);
    Rt.at<double>(2,3) = ksi.at<double>(5);
#endif
}

static inline
void calcRgbdEquationCoeffs(double* C, double dIdx, double dIdy, const Point3f& p3d, double fx, double fy)
{
    double invz  = 1. / p3d.z,
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
void calcICPEquationCoeffs(double* C, const Point3f& p0, const Vec3f& n1)
{
    C[0] = -p0.z * n1[1] + p0.y * n1[2];
    C[1] =  p0.z * n1[0] - p0.x * n1[2];
    C[2] = -p0.y * n1[0] + p0.x * n1[1];
    C[3] = n1[0];
    C[4] = n1[1];
    C[5] = n1[2];
}

static inline
void calcFeatureXEquationCoeffs(double* C, const Point3f& p3d, double fx)
{
    double invz  = 1. / p3d.z;

    C[0] = -(fx * p3d.x * p3d.y * invz * invz);
    C[1] = fx + fx * p3d.x * p3d.x * invz * invz;
    C[2] = -(fx * p3d.y * invz);
    C[3] = fx * invz;
    C[4] = 0;
    C[5] = -(fx * p3d.x * invz * invz);
}

static inline
void calcFeatureYEquationCoeffs(double* C, const Point3f& p3d, double fy)
{
    double invz  = 1. / p3d.z;

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
    //const float icpWeight = 10.0;

    std::vector<Mat> pyramidCameraMatrix;

    Mat resultRt = initRt.empty() ? Mat::eye(4,4,CV_64FC1) : initRt.clone();
    Mat currRt, ksi;

    bool isOk = false;
    {
        const Mat& levelCameraMatrix = cameraMatrix;
        const Mat& levelCameraMatrix_inv = levelCameraMatrix.inv(DECOMP_SVD);
        const Mat& srcLevelDepth = srcFrame->depth;
        const Mat& dstLevelDepth = dstFrame->depth;

        const double fx = levelCameraMatrix.at<double>(0,0);
        const double fy = levelCameraMatrix.at<double>(1,1);
        const double cx = levelCameraMatrix.at<double>(0,2);
        const double cy = levelCameraMatrix.at<double>(1,2);
        const double determinantThreshold = 1e-6;

        Mat AtA_rgbd, AtB_rgbd, AtA_icp, AtB_icp;
        Mat corresps_rgbd, corresps_icp;

        // Run transformation search on current level iteratively.
        for(int iter = 0; iter < iterCounts_vec[0]; iter ++)
        {
            Mat AtA(transformDim, transformDim, CV_64FC1, Scalar(0)), AtB(transformDim, 1, CV_64FC1, Scalar(0));
            if(iter>=feature_iter_num){
                Mat resultRt_inv = resultRt.inv(DECOMP_SVD);

                int v_rgbd = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, 
                                             //resultRt_inv, srcLevelDepth, srcFrame->maskDepth, dstLevelDepth, dstFrame->maskText,
                                             resultRt, dstLevelDepth, dstFrame->maskDepth, srcLevelDepth, srcFrame->maskText,
                                             maxDepthDiff, corresps_rgbd);
                if (v_rgbd > v_max)
                    v_max = v_rgbd;
                //cout << "v_rgbd" << v_rgbd << endl;
                //exit(1);
                int v_icp = computeCorresps(levelCameraMatrix, levelCameraMatrix_inv, 
                                            //resultRt_inv, srcLevelDepth, srcFrame->maskDepth, dstLevelDepth, dstFrame->maskNormal,
                                            resultRt, dstLevelDepth, dstFrame->maskDepth, srcLevelDepth, srcFrame->maskNormal,
                                            maxDepthDiff, corresps_icp);
                
                if (v_icp > v_max)
                    v_max = v_icp;
                //cout << "v_icp" << v_icp << endl;

                if(corresps_rgbd.rows >= minCorrespsCount)
                {
                    calcRgbdLsmMatrices(srcFrame->image, srcFrame->cloud, resultRt,
                                        dstFrame->image, dstFrame->dI_dx, dstFrame->dI_dy,
                                        corresps_rgbd, fx, fy, sobelScale,
                                        AtA_rgbd, AtB_rgbd, rgbdEquationFuncPtr, transformDim);

                    AtA += AtA_rgbd;
                    AtB += AtB_rgbd;
                }

                if(corresps_icp.rows >= minCorrespsCount)
                {
                    calcICPLsmMatrices(srcFrame->cloud, resultRt,
                                       dstFrame->cloud, dstFrame->normals,
                                       corresps_icp, AtA_icp, AtB_icp, icpEquationFuncPtr, transformDim);
                    AtA += AtA_icp;
                    AtB += AtB_icp;
                }
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
               //cout << "AtA " << AtA << endl;
               //cout << "AtB " << AtB << endl;
               //cout << "cloud " << srcFrame->cloud << endl;
               //cout << "fx " << fx << endl;
               //cout << "iter " << iter << endl;
               //exit(1);
            }

            bool solutionExist = solveSystem(AtA, AtB, determinantThreshold, ksi);
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


