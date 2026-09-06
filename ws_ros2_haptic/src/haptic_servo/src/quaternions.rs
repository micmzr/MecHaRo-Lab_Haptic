use std::fmt; 

#[derive(Debug, Clone, Default)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl From<ros_env::geometry_msgs::msg::Quaternion> for Quaternion {
    fn from(q: ros_env::geometry_msgs::msg::Quaternion) -> Self {
        Self {
            x: q.x,
            y: q.y,
            z: q.z,
            w: q.w,
        }
    }
}

impl fmt::Display for Quaternion {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        // Use `self.number` to refer to each positional data point.
        write!(f, "(x : {}, y : {}, z : {}, w : {})", self.x, self.y, self.z, self.w)
    }
}


impl Quaternion {
    pub fn to_ros_msg(&self) -> ros_env::geometry_msgs::msg::Quaternion {
        ros_env::geometry_msgs::msg::Quaternion {
            x: self.x,
            y: self.y,
            z: self.z,
            w: self.w,
        }
    }

    #[allow(unused)]
    pub fn multiply(&self, other: &Quaternion) -> Quaternion {
        Quaternion {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }

    #[allow(unused)]
    pub fn inverse(&self) -> Quaternion {
        let norm_sq = self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w;
        Quaternion {
            w: self.w / norm_sq,
            x: -self.x / norm_sq,
            y: -self.y / norm_sq,
            z: -self.z / norm_sq,
        }
    }

    #[allow(unused)]
    pub fn delta(&self, other: &Quaternion) -> Quaternion {
        self.multiply(&other.inverse())
    }

    #[allow(unused)]
    pub fn to_rpy(&self) -> (f64, f64, f64) {
        let roll = (2.0 * (self.w * self.x + self.y * self.z)).atan2(1.0 - 2.0 * (self.x * self.x + self.y * self.y));
        let pitch = (2.0 * (self.w * self.y - self.z * self.x)).asin();
        let yaw = (2.0 * (self.w * self.z + self.x * self.y)).atan2(1.0 - 2.0 * (self.y * self.y + self.z * self.z));
        (roll, pitch, yaw)
    }

    #[allow(unused)]
    pub fn from_rpy(roll: f64, pitch: f64, yaw: f64) -> Quaternion {
        let cy = (yaw * 0.5).cos();
        let sy = (yaw * 0.5).sin();
        let cp = (pitch * 0.5).cos();
        let sp = (pitch * 0.5).sin();
        let cr = (roll * 0.5).cos();
        let sr = (roll * 0.5).sin();

        Quaternion {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }
}
