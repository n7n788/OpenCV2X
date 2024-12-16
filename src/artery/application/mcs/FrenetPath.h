#ifndef ARTERY_FRENETPATH_H
#define ARTERY_FRENETPATH_H

#include "QuinticPolynomial.h"
#include "QuarticPolynomial.h"
#include <vector>

namespace artery
{

class FrenetPath
{
public:

	static constexpr double TIME_STEP = 0.1; // 経路のタイムステップ
	static constexpr double TIME_LENGTH = 5.0; // 経路の時間長

	// 1次元の経路  
	struct Path {
		std::vector<double> mLonPoses; // 縦方向の位置
		std::vector<double> mLonSpeeds; // 縦方向の速度
		std::vector<double> mLonAccels; // 縦方向の加速度
		std::vector<double> mLonJarks; // 縦方向のジャーク
	};

	// getter
	const mlonPath& getLonPath { return mLonPath; }
	const mLatPath& getLatPath { return mLatPath; }
	double getCost() { return mCost; }

	// setter
	void setLonPath(const Path& lonPath) { mLonPath = lonPath; }
	void setLatPath(const Path& latPath) { mLatPath = latPath; }

	// 5次元方程式から経路情報を生成
	// @param qp [QuinticPolynomial] 5次元方程式
	void generateFrenetPath(const QuinticPolynomial& qp);

	// 4次元方程式から経路情報を生成
	// @param qp [QuarticPolynomial] 4次元方程式
	void generateFrenetPath(const QuarticPolynomial& qp);

	// 経路のコストを計算
	void calculateCost();

private:
	Path mlonPath;　// 縦方向のパス
	Path mlatPath; // 横方向のパス
	double mCost; // 経路のコスト
};
}
#endif /* ARTERY_FRENETPATH_H */
