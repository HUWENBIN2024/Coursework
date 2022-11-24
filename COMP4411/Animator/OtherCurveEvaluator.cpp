#include "OtherCurveEvaluator.h"
#include "vec.h"
#include "mat.h"
#include "modelerapp.h"
#include "modelerglobals.h"
#include <iostream>
#include "Eigen/Dense"
#include "modelerapp.h"
using namespace Eigen;
using namespace std;

const Mat4d BezierMatrix(-1, 3, -3, 1, 3, -6, 3, 0, -3, 3, 0, 0, 1, 0, 0, 0);
const Mat4d CatRomMatrix(-1, 3, -3, 1, 2, -5, 4, -1, -1, 0, 1, 0, 0, 2, 0, 0);

void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap, std::map<Point, Point*>innerPts) const {

	if (ptvCtrlPts.empty()) return;
	ptvEvaluatedCurvePts.clear();
	int size = ptvCtrlPts.size();

	int outlierCount = bWrap ? size % 3 : (size - 1) % 3;
	int EndIndex;
	switch (size % 3) {
	case 0: EndIndex = size - 6; break;
	case 1: EndIndex = size - 4; break;
	case 2: EndIndex = size - 5; break;
	}

	for (int i = 0; 3*i <= EndIndex; ++i) {
		// General Case
		Point P[5] = { Point(0, 0),ptvCtrlPts[3 * i],ptvCtrlPts[3 * i + 1],ptvCtrlPts[3 * i + 2],ptvCtrlPts[3 * i + 3] };
		Vec4d X(P[1].x, P[2].x, P[3].x, P[4].x);
		Vec4d Y(P[1].y, P[2].y, P[3].y, P[4].y);
		for (double t = 0; t < 1; t+=0.05) {
			Vec4d T(t * t * t, t * t, t, 1);
			Point p(T * (BezierMatrix * X), T * (BezierMatrix * Y));
			ptvEvaluatedCurvePts.push_back(p);
		}
		// ptvEvaluatedCurvePts.push_back(P[4]);
	}
	// Special Case
	if (bWrap) {
		switch (outlierCount) {
		case 0: {
			Point p4(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y);
			Point P[5] = { Point(0, 0),ptvCtrlPts[size-3],ptvCtrlPts[size-2],ptvCtrlPts[size-1],p4 };
			Vec4d X(P[1].x, P[2].x, P[3].x, P[4].x);
			Vec4d Y(P[1].y, P[2].y, P[3].y, P[4].y);
			bool flag = true;
			Point prev;
			for (double t = 0; t <= 1; t = min(t+0.05,1.0)) {
				Vec4d T(t * t * t, t * t, t, 1);
				double px = T * (BezierMatrix * X);
				Point p(px, T * (BezierMatrix * Y));
				if (px >= fAniLength) {
					p.x -= fAniLength;
					if (flag) {
						flag = false;
						double y1 = (p.y * (fAniLength - prev.x) +
							prev.y * p.x) /
							(p.x + fAniLength - prev.x);
						ptvEvaluatedCurvePts.push_back(Point(0, y1));
						ptvEvaluatedCurvePts.push_back(Point(fAniLength, y1));
					}
				}
				prev = p;
				if (t == 1) break;
				ptvEvaluatedCurvePts.push_back(p);
			}
			break;
		}
		case 1: {
			ptvEvaluatedCurvePts.push_back(ptvCtrlPts[size - 1]);
			double y1 = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[size - 1].x) +
				ptvCtrlPts[size - 1].y * ptvCtrlPts[0].x) /
				(ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[size - 1].x);
			ptvEvaluatedCurvePts.push_back(Point(0, y1));
			ptvEvaluatedCurvePts.push_back(Point(fAniLength, y1));
			break;
		}
		case 2: {
			ptvEvaluatedCurvePts.push_back(ptvCtrlPts[size - 2]);
			ptvEvaluatedCurvePts.push_back(ptvCtrlPts[size - 1]);
			double y1 = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[size - 1].x) +
				ptvCtrlPts[size - 1].y * ptvCtrlPts[0].x) /
				(ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[size - 1].x);
			ptvEvaluatedCurvePts.push_back(Point(0, y1));
			ptvEvaluatedCurvePts.push_back(Point(fAniLength, y1));
			break;
		}
		}
	}
	else {
		for (int i = outlierCount+1; i > 0; --i) {
			ptvEvaluatedCurvePts.push_back(ptvCtrlPts[size - i]);
		}
		ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[size-1].y));
	}
}

void BSplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap, std::map<Point, Point*>innerPts) const {
	if (ptvCtrlPts.empty()) 
		return;
	std::vector <Point> tempBezierPts;
	Mat4d BsplineMatrix(1,4,1,0, 0,4,2,0, 0,2,4,0, 0,1,4,1);
	int CtrlPtsSize = ptvCtrlPts.size();
	if (CtrlPtsSize < 4)
		return;

	Point deBoorPoint_0[4] = { ptvCtrlPts[0], ptvCtrlPts[1], ptvCtrlPts[2], ptvCtrlPts[3] };
	Vec4d X(deBoorPoint_0[0].x, deBoorPoint_0[1].x, deBoorPoint_0[2].x, deBoorPoint_0[3].x);
	Vec4d Y(deBoorPoint_0[0].y, deBoorPoint_0[1].y, deBoorPoint_0[2].y, deBoorPoint_0[3].y);
	Vec4d V_X_0((BsplineMatrix / 6.0) * X);
	Vec4d V_Y_0((BsplineMatrix / 6.0) * Y);
	Point BezierCtrlPoint_0(V_X_0[0], V_Y_0[0]);
	tempBezierPts.push_back(BezierCtrlPoint_0);

	for (int i = 0; i < CtrlPtsSize - 3; i++)
	{
		Point deBoorPoint[4] = { ptvCtrlPts[i], ptvCtrlPts[i+1], ptvCtrlPts[i+2], ptvCtrlPts[i+3] };
		Vec4d X(deBoorPoint[0].x, deBoorPoint[1].x, deBoorPoint[2].x, deBoorPoint[3].x);
		Vec4d Y(deBoorPoint[0].y, deBoorPoint[1].y, deBoorPoint[2].y, deBoorPoint[3].y);
		// b(i to i+3) to V(0,1,2,3) by matrix  multiplication
		Vec4d V_X((BsplineMatrix / 6.0) * X );
		Vec4d V_Y((BsplineMatrix / 6.0) * Y );
		for (int j = 1; j < 4; ++j)
		{
			Point BezierCtrlPoint(V_X[j], V_Y[j]);
			tempBezierPts.push_back(BezierCtrlPoint);
		}
	}
	BezierCurveEvaluator tempEvaluator;
	tempEvaluator.evaluateCurve(tempBezierPts, ptvEvaluatedCurvePts, fAniLength, bWrap, innerPts);
}

void CatmullRomCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap, std::map<Point, Point*>innerPts) const {

	if (ptvCtrlPts.empty()) return;
	ptvEvaluatedCurvePts.clear();
	int size = ptvCtrlPts.size();

	double tau = VAL(CR_TENSION);
	Point P[5];
	Vec4d X, Y;
	double Max_x = -1;
	std::map<Point, Point*> ip = innerPts;

	// Case size==2
	if (size == 2) {
		ptvEvaluatedCurvePts.push_back(ptvCtrlPts[0]);
		ptvEvaluatedCurvePts.push_back(ptvCtrlPts[1]);
		if (bWrap) {
			double y1 = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[1].x) +
				ptvCtrlPts[1].y * ptvCtrlPts[0].x) /
				(ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[1].x);
			ptvEvaluatedCurvePts.push_back(Point(0, y1));
			ptvEvaluatedCurvePts.push_back(Point(fAniLength, y1));
		}
		else {
			ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
			ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[1].y));
		}
		return;
	}

	// Leftmost
	Max_x = -1;
	P[1] = ptvCtrlPts[0];
	P[2] = bWrap? P[1] + tau / 3 * (ptvCtrlPts[1] - Point(ptvCtrlPts[size-1].x-fAniLength,ptvCtrlPts[size-1].y))
																:P[1] + tau / 3 * (ptvCtrlPts[1] - ptvCtrlPts[0]);
	if (VAL(EDITCR)) P[2] = ip.find(P[1])->second[1];
	P[4] = ptvCtrlPts[1];
	P[3] = ptvCtrlPts[1] - tau / 3 * (ptvCtrlPts[2] - ptvCtrlPts[0]);
	if (VAL(EDITCR)) P[3] = ip.find(P[4])->second[0];
	X = Vec4d(P[1].x, P[2].x, P[3].x, P[4].x);
	Y = Vec4d(P[1].y, P[2].y, P[3].y, P[4].y);
	for (double t = 0; t < 1; t += 0.05) {
		Vec4d T(t * t * t, t * t, t, 1);
		Point p(T * (BezierMatrix * X), T * (BezierMatrix * Y));
		if (p.x >= Max_x) {
			ptvEvaluatedCurvePts.push_back(p);
			Max_x = p.x;
		}
	}
	// General Case
	for (int i = 1; i <= size - 3; ++i) {
		P[1] = ptvCtrlPts[i];
		P[2] = P[1] + tau / 3 * (ptvCtrlPts[i + 1] - ptvCtrlPts[i - 1]);
		if (VAL(EDITCR)) P[2] = ip.find(P[1])->second[1];
		P[4] = ptvCtrlPts[i + 1];
		P[3] = ptvCtrlPts[i + 1] - tau / 3 * (ptvCtrlPts[i + 2] - ptvCtrlPts[i]);
		if (VAL(EDITCR)) P[3] = ip.find(P[4])->second[0];
		X = Vec4d(P[1].x, P[2].x, P[3].x, P[4].x);
		Y = Vec4d(P[1].y, P[2].y, P[3].y, P[4].y);
		for (double t = 0; t < 1; t += 0.05) {
			Vec4d T(t * t * t, t * t, t, 1);
			Point p(T * (BezierMatrix * X), T * (BezierMatrix * Y));
			if (p.x >= Max_x) {
				ptvEvaluatedCurvePts.push_back(p);
				Max_x = p.x;
			}
		}
	}
	// Rightmost
	P[1] = ptvCtrlPts[size-2];
	P[2] = P[1] + tau / 3 * (ptvCtrlPts[size-1] - ptvCtrlPts[size-3]);
	if (VAL(EDITCR)) P[2] = ip.find(P[1])->second[1];
	P[4] = ptvCtrlPts[size - 1];
	P[3] = bWrap ? ptvCtrlPts[size-1] - tau / 3 * (Point(ptvCtrlPts[0].x+fAniLength, ptvCtrlPts[0].y) - ptvCtrlPts[size-2])
		: ptvCtrlPts[size-1] - tau / 3 * (ptvCtrlPts[size-1] - ptvCtrlPts[size-2]);
	if (VAL(EDITCR)) P[3] = ip.find(P[4])->second[0];
	X = Vec4d(P[1].x, P[2].x, P[3].x, P[4].x);
	Y = Vec4d(P[1].y, P[2].y, P[3].y, P[4].y);
	for (double t = 0; t < 1; t += 0.05) {
		Vec4d T(t * t * t, t * t, t, 1);
		Point p(T * (BezierMatrix * X), T * (BezierMatrix * Y));
		if (p.x >= Max_x) {
			ptvEvaluatedCurvePts.push_back(p);
			Max_x = p.x;
		}
	}
	// Boundary
	if (bWrap) {
		if (ptvCtrlPts[0].x == 0 && ptvCtrlPts[size - 1].x == fAniLength) return;
		P[0] = Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y);
		P[1] = ptvCtrlPts[size - 1];
		P[2] = P[1] + tau / 3 * (P[0] - ptvCtrlPts[size - 2]);
		P[3] = P[0] - tau / 3 * (Point(ptvCtrlPts[1].x + fAniLength, ptvCtrlPts[1].y) - P[1]);
		P[4] = P[0];
		X = Vec4d(P[1].x, P[2].x, P[3].x, P[4].x);
		Y = Vec4d(P[1].y, P[2].y, P[3].y, P[4].y);
		bool flag = true;
		Point prev;
		for (double t = 0; t <= 1; t = min(t + 0.05, 1.0)) {
			Vec4d T(t * t * t, t * t, t, 1);
			double px = T * (BezierMatrix * X);
			Point p(px, T * (BezierMatrix * Y));
			if (px >= fAniLength) {
				p.x -= fAniLength;
				if (flag) {
					flag = false;
					double y1 = (p.y * (fAniLength - prev.x) +
							prev.y * p.x) /
							(p.x + fAniLength - prev.x);
					if (ptvCtrlPts[0].x != 0) {
						ptvEvaluatedCurvePts.push_back(Point(0, y1));
					}
					ptvEvaluatedCurvePts.push_back(Point(fAniLength, y1));
				}
			}
			if (t == 1) break;
			if (px > prev.x && px >= Max_x && px <= P[0].x) {
				prev = p;
				Max_x = px;
				ptvEvaluatedCurvePts.push_back(p);
			}
			if (px > prev.x && px >= Max_x && px > P[0].x) {
				Max_x = px;
			}
		}
	}
	else {
		if (ptvCtrlPts[size - 1].x >= Max_x) {
			ptvEvaluatedCurvePts.push_back(ptvCtrlPts[size - 1]);
		}
		else {
			ptvEvaluatedCurvePts.push_back(Point(Max_x, ptvCtrlPts[size - 1].y));
		}
		ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[size - 1].y));
	}
}

void C2InterpolatingCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap, std::map<Point, Point*>innerPts) const {
	int m = ptvCtrlPts.size() - 1;  // m is the number of total C2 ctrl points
	MatrixXd M(m+1, m+1);
	for (int i = 0; i < m + 1; i++)
	{
		for (int j = 0; j < m + 1; j++)
		{
			M(i, j) = 0;
		}
	}
	M(0, 0) = 2; M(0, 1) = 1;
	M(m, m-1) = 1; M(m, m) = 2;
	for (int i = 1; i < m; i++)
	{
		M(i, i - 1) = 1;
		M(i, i) = 4;
		M(i, i + 1) = 1;
	}
	
	MatrixXd M_inverse = M.inverse();
	
	VectorXd F_X(m + 1);
	F_X(0) = 3 * (ptvCtrlPts[1].x - ptvCtrlPts[0].x);
	F_X(m) = 3 * (ptvCtrlPts[m].x - ptvCtrlPts[m-1].x);
	for (int i = 1; i < m; i++)
	{
		F_X(i) = 3*(ptvCtrlPts[i+1].x - ptvCtrlPts[i-1].x);
	}

	VectorXd F_Y(m + 1);
	F_Y(0) = 3 * (ptvCtrlPts[1].y - ptvCtrlPts[0].y);
	F_Y(m) = 3 * (ptvCtrlPts[m].y - ptvCtrlPts[m - 1].y);
	for (int i = 1; i < m; i++)
	{
		F_Y(i) = 3 * (ptvCtrlPts[i + 1].y - ptvCtrlPts[i - 1].y);
	}

	MatrixXd D_X = M_inverse * F_X;
	MatrixXd D_Y = M_inverse * F_Y;

	vector<Point> tempBezierPts;
	tempBezierPts.push_back(ptvCtrlPts[0]);
 	for (int i = 0; i < m; i++)
	{
		Point start = ptvCtrlPts[i];
		Point end = ptvCtrlPts[i + 1];
		Point start_d(D_X(i), D_Y(i));
		Point end_d(D_X(i + 1), D_Y(i + 1));

		tempBezierPts.push_back(Point(start.x + start_d.x / 3.0, start.y + start_d.y / 3.0));
		tempBezierPts.push_back(Point(end.x - end_d.x / 3.0, end.y - end_d.y / 3.0));
		tempBezierPts.push_back(Point(end.x, end.y));
	}
	BezierCurveEvaluator tempEvaluator;
	tempEvaluator.evaluateCurve(tempBezierPts, ptvEvaluatedCurvePts, fAniLength, bWrap, innerPts);
	
}


void SubdivisionCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap, std::map<Point, Point*>innerPts) const {
	
}


