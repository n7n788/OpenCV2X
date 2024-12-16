#ifndef ARTERY_FRENETPATH_H
#define ARTERY_FRENETPATH_H

#include <vector>

namespace artery
{

class FrenetPath
{
public:

	// getter
	const mlonPath& getLonPath { return mLonPath; }
	const mLatPath& getLatPath { return mLatPath; }
	double getCost() { return mCost; }

	// setter
	void setLonPath(const Path& lonPath) { mLonPath = lonPath; }
	void setLatPath(const Path& latPath) { mLatPath = latPath; }

	// 経路のコストを計算
	void calculateCost();

private:
	Path mlonPath;　// 縦方向のパス
	Path mlatPath; // 横方向のパス
	double mCost; // 経路のコスト
};
}
#endif /* ARTERY_FRENETPATH_H */
