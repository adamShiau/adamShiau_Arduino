#pragma once

#include "data_types.h"
#include <stdint.h>

class CoordTransform {
public:
    CoordTransform();
    ~CoordTransform();
    
    // Initialize with local origin (for ENU transformations)
    bool setLocalOrigin(double lat_origin, double lon_origin, float alt_origin);
    
    // Coordinate transformations
    void llhToEnu(double lat, double lon, float alt, float& east, float& north, float& up);
    void enuToLlh(float east, float north, float up, double& lat, double& lon, float& alt);
    
    void llhToEcef(double lat, double lon, float alt, float& x, float& y, float& z);
    void ecefToLlh(float x, float y, float z, double& lat, double& lon, float& alt);
    
    void enuToNed(float east, float north, float up, float& north_ned, float& east_ned, float& down_ned);
    void nedToEnu(float north_ned, float east_ned, float down_ned, float& east, float& north, float& up);
    
    // Rotation matrix transformations
    void bodyToEnu(const float accel_body[3], const float quaternion[4], float accel_enu[3]);
    void enuToBody(const float accel_enu[3], const float quaternion[4], float accel_body[3]);
    
    // Quaternion operations
    void quaternionToRotationMatrix(const float quaternion[4], float rotation_matrix[3][3]);
    void rotationMatrixToQuaternion(const float rotation_matrix[3][3], float quaternion[4]);
    void quaternionToEuler(const float quaternion[4], float& roll, float& pitch, float& yaw);
    void eulerToQuaternion(float roll, float pitch, float yaw, float quaternion[4]);
    
    // Quaternion math utilities
    void quaternionMultiply(const float q1[4], const float q2[4], float result[4]);
    void quaternionConjugate(const float quaternion[4], float conjugate[4]);
    void quaternionNormalize(float quaternion[4]);
    float quaternionNorm(const float quaternion[4]);
    
    // Velocity transformations
    void transformVelocityLlhToEnu(double lat, double lon, float vel_north, float vel_east, float vel_up,
                                  float& vel_enu_x, float& vel_enu_y, float& vel_enu_z);
    
    // Utility functions
    bool isValidLatLon(double lat, double lon);
    bool isValidAltitude(float alt);
    float calculateDistance(double lat1, double lon1, double lat2, double lon2);
    float calculateBearing(double lat1, double lon1, double lat2, double lon2);
    
    // Get current origin
    void getLocalOrigin(double& lat_origin, double& lon_origin, float& alt_origin) const;
    bool hasValidOrigin() const { return origin_set_; }
    
private:
    // Local ENU origin
    double lat_origin_;
    double lon_origin_;
    float alt_origin_;
    bool origin_set_;
    
    // Cached trigonometric values for origin
    double sin_lat_origin_;
    double cos_lat_origin_;
    double sin_lon_origin_;
    double cos_lon_origin_;
    
    // Earth model constants (WGS84)
    static constexpr double EARTH_SEMI_MAJOR_AXIS = 6378137.0;
    static constexpr double EARTH_SEMI_MINOR_AXIS = 6356752.314245;
    static constexpr double EARTH_ECCENTRICITY_SQUARED = 0.00669437999014;
    static constexpr double EARTH_FLATTENING = 1.0 / 298.257223563;
    
    // Internal helper functions
    double calculateEarthRadius(double latitude);
    double calculateMeridionalRadius(double latitude);
    double calculateNormalRadius(double latitude);
    
    void updateOriginCache();
    
    // Matrix operations
    void matrixMultiply3x3(const float a[3][3], const float b[3][3], float result[3][3]);
    void matrixVectorMultiply3x3(const float matrix[3][3], const float vector[3], float result[3]);
    void matrixTranspose3x3(const float matrix[3][3], float result[3][3]);
};