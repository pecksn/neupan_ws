/**
 * @file collisiondetection.cpp
 * @brief 碰撞检测函数集
 * @date 2019-12-12
 * 
 * 
 */
#include "collisiondetection.h"

using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr;
  Lookup::collisionLookup(collisionLookup);
}

bool CollisionDetection::configurationTest(float x, float y, float t) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(t)) return false;
  if (poseValidator) return poseValidator(x, y, t);
  if (!grid) return false;
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);//得出X方向在cell中的偏移量
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);//Y方向在cell中的偏移量
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  if (iT < 0 || iT >= Constants::headings) return false;
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  int cX;
  int cY;

  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      if (grid->data[cY * grid->info.width + cX]) {
        return false;
      }//若grid的某个小网格存在值，说明有障碍，则返回false表示不在自由网格
    } else {
      return false;  // a footprint outside the map is not observed free space
    }
  }

  return true;//所有检测都没有检测到被占用，说明没有障碍，可以通行
}
