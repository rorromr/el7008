#ifndef OBJECT_PSD_SAMPLE_SUPERQUADRIC_H
#define OBJECT_PSD_SAMPLE_SUPERQUADRIC_H

#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

namespace object_psd
{
  template<typename Scalar>
  struct SuperquadricParameters
  {
    SuperquadricParameters ()
      : e1 (0.), e2 (0.), a (1.), b (1.), c (1.)
      , transform (Eigen::Matrix<Scalar, 4, 4>::Identity ())
    {}

    Scalar e1, e2, a, b, c;
    Eigen::Matrix<Scalar, 4, 4> transform;
  };

  /**
   * Naive superquadric sampling (i.e., uniformly sample the parameter space)
   */
  template <typename PointT, typename Scalar>
  class SuperquadricSampling
  {
  public:
    SuperquadricSampling ()
    {
      eta_samples_ = 100;
      mu_samples_ = 100;
    }

    inline void
    setParameters (const SuperquadricParameters<Scalar> &params)
    {
      params_ = params;
    }

    inline void
    setSampleCount (const int eta_samples,
                    const int mu_samples)
    {
      eta_samples_ = eta_samples;
      mu_samples_ = mu_samples;
    }

    void
    generatePointCloud (pcl::PointCloud<PointT> &cloud)
    {
      Scalar eta_sample_rate = static_cast<Scalar> (M_PI) / static_cast<Scalar> (eta_samples_);
      Scalar mu_sample_rate = static_cast<Scalar> (2*M_PI) / static_cast<Scalar> (mu_samples_);

      for (double eta = -M_PI / 2.0; eta < M_PI / 2.; eta += eta_sample_rate)
      {
        for (double mu = -M_PI; mu < M_PI; mu += mu_sample_rate)
        {
          Eigen::Matrix<Scalar, 4, 1> p;
          p[0] = params_.a * (cos(eta) / fabs(cos(eta))) * pow (fabs(cos (eta)), params_.e1) * (cos(mu) / fabs(cos(mu))) * pow (fabs (cos (mu)), params_.e2);
          p[1] = params_.b * (cos(eta) / fabs(cos(eta))) * pow (fabs(cos (eta)), params_.e1) * (sin(mu) / fabs(sin(mu))) * pow (fabs (sin (mu)), params_.e2);
          p[2] = params_.c * (sin(eta) / fabs(sin(eta))) * pow (fabs(sin (eta)), params_.e1);
          p[3] = 1.;

          PointT point;
          Eigen::Matrix<Scalar, 4, 1> p_tr = params_.transform.inverse () * p;
          point.x = p_tr[0];
          point.y = p_tr[1];
          point.z = p_tr[2];

          if (isFinite (point))
            cloud.push_back(point);
        }
      }
      cloud.height = 1.;
      cloud.width = cloud.size ();
    }

    void
    generateMesh (pcl::PolygonMesh &mesh)
    {
      // First generate the point cloud
      pcl::PointCloud<PointT> cloud;
      generatePointCloud(cloud);


      // Then look into the connectivity
      for (size_t v = 1; v < eta_samples_; ++v)
      {
        for (size_t u = 1; u < mu_samples_; ++u)
        {
          pcl::Vertices polygon;
          polygon.vertices.push_back(v * eta_samples_ + u);
          polygon.vertices.push_back(v * eta_samples_ + u - 1);
          polygon.vertices.push_back((v - 1) * eta_samples_ + u);
          mesh.polygons.push_back(polygon);

          polygon.vertices.clear ();
          polygon.vertices.push_back((v - 1) * eta_samples_ + u);
          polygon.vertices.push_back(v * eta_samples_ + u - 1);
          polygon.vertices.push_back((v - 1) * eta_samples_ + u - 1);
          mesh.polygons.push_back(polygon);
        }

        // And connect the last column with the first one
        pcl::Vertices polygon;
        polygon.vertices.push_back(v * eta_samples_ + 0);
        polygon.vertices.push_back(v * eta_samples_ + eta_samples_ - 1);
        polygon.vertices.push_back((v - 1) * eta_samples_ + 0);
        mesh.polygons.push_back(polygon);

        polygon.vertices.clear ();
        polygon.vertices.push_back((v - 1) * eta_samples_ + 0);
        polygon.vertices.push_back(v * eta_samples_ + eta_samples_ - 1);
        polygon.vertices.push_back((v - 1) * eta_samples_ + eta_samples_ - 1);
        mesh.polygons.push_back(polygon);
      }

      /// TODO
      /*
      /// Take care of the top: add a new point as the centroid of the first row
      /// and do a triangle strip there
      Eigen::Vector3f centroid (0.f, 0.f, 0.f);
      for (size_t i = 0; i < eta_samples_; ++i)
        centroid += cloud[i].getVector3fMap ();
      centroid /= mu_samples_;
      PointT point_centroid;
      point_centroid.getVector3fMap () = centroid;
      cloud.push_back(point_centroid);

      for (size_t i = 1; i < eta_samples_; ++i)
      {
        pcl::Vertices polygon;
        polygon.vertices.push_back(cloud.size () - 1);
        polygon.vertices.push_back(i - 1);
        polygon.vertices.push_back(i);

        mesh.polygons.push_back(polygon);
      }
      pcl::Vertices polygon;
      polygon.vertices.push_back(cloud.size () - 1);
      polygon.vertices.push_back(mu_samples_);
      polygon.vertices.push_back(0);
      mesh.polygons.push_back(polygon);
      */
      pcl::toPCLPointCloud2(cloud, mesh.cloud);
    }



  protected:
    SuperquadricParameters<Scalar> params_;
    int eta_samples_;
    int mu_samples_;

  }; /* SuperquadricSampling class */


} /* object_psd namespace */

#endif /* OBJECT_PSD_SAMPLE_SUPERQUADRIC_H header guard */
