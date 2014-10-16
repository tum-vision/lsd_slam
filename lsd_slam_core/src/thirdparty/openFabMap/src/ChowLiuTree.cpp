/*------------------------------------------------------------------------
 Copyright 2012 Arren Glover [aj.glover@qut.edu.au]
                Will Maddern [w.maddern@qut.edu.au]

 This file is part of OpenFABMAP. http://code.google.com/p/openfabmap/

 OpenFABMAP is free software: you can redistribute it and/or modify it under
 the terms of the GNU General Public License as published by the Free Software
 Foundation, either version 3 of the License, or (at your option) any later
 version.

 OpenFABMAP is distributed in the hope that it will be useful, but WITHOUT ANY
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 details.

 For published work which uses all or part of OpenFABMAP, please cite:
 http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6224843

 Original Algorithm by Mark Cummins and Paul Newman:
 http://ijr.sagepub.com/content/27/6/647.short
 http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=5613942
 http://ijr.sagepub.com/content/30/9/1100.abstract

 You should have received a copy of the GNU General Public License along with
 OpenFABMAP. If not, see http://www.gnu.org/licenses/.
------------------------------------------------------------------------*/

#include "../include/openfabmap.hpp"

using std::vector;
using std::list;
using std::map;
using cv::Mat;

namespace of2 {

ChowLiuTree::ChowLiuTree() {
}

ChowLiuTree::~ChowLiuTree() {
}

void ChowLiuTree::add(const Mat& imgDescriptor) {
	CV_Assert(!imgDescriptor.empty());
	if (!imgDescriptors.empty()) {
		CV_Assert(imgDescriptors[0].cols == imgDescriptor.cols);
		CV_Assert(imgDescriptors[0].type() == imgDescriptor.type());
	}

	imgDescriptors.push_back(imgDescriptor);

}

void ChowLiuTree::add(const vector<Mat>& imgDescriptors) {
	for (size_t i = 0; i < imgDescriptors.size(); i++) {
		add(imgDescriptors[i]);
	}
}

const std::vector<cv::Mat>& ChowLiuTree::getImgDescriptors() const {
	return imgDescriptors;
}

Mat ChowLiuTree::make(double infoThreshold) {
	CV_Assert(!imgDescriptors.empty());

	unsigned int descCount = 0;
	for (size_t i = 0; i < imgDescriptors.size(); i++)
		descCount += imgDescriptors[i].rows;

	mergedImgDescriptors = cv::Mat(descCount, imgDescriptors[0].cols, 
		imgDescriptors[0].type());
	for (size_t i = 0, start = 0; i < imgDescriptors.size(); i++)
	{
		Mat submut = mergedImgDescriptors.rowRange((int)start, 
			(int)(start + imgDescriptors[i].rows));
		imgDescriptors[i].copyTo(submut);
		start += imgDescriptors[i].rows;
	}

	list<info> edges;
	createBaseEdges(edges, infoThreshold);

	// TODO: if it cv_asserts here they really won't know why.

	CV_Assert(reduceEdgesToMinSpan(edges));

	return buildTree(edges.front().word1, edges);
}

double ChowLiuTree::P(int a, bool za) {

	if(za) {
		return (0.98 * cv::countNonZero(mergedImgDescriptors.col(a)) / 
			mergedImgDescriptors.rows) + 0.01;
	} else {
		return 1 - ((0.98 * cv::countNonZero(mergedImgDescriptors.col(a)) / 
			mergedImgDescriptors.rows) + 0.01);
	}

}
double ChowLiuTree::JP(int a, bool za, int b, bool zb) {

	double count = 0;
	for(int i = 0; i < mergedImgDescriptors.rows; i++) {
		if((mergedImgDescriptors.at<float>(i,a) > 0) == za && 
			(mergedImgDescriptors.at<float>(i,b) > 0) == zb) {
				count++;
		}
	}
	return count / mergedImgDescriptors.rows;

}
double ChowLiuTree::CP(int a, bool za, int b, bool zb){

	int count = 0, total = 0;
	for(int i = 0; i < mergedImgDescriptors.rows; i++) {
		if((mergedImgDescriptors.at<float>(i,b) > 0) == zb) {
			total++;
			if((mergedImgDescriptors.at<float>(i,a) > 0) == za) {
				count++;
			}
		}
	}
	if(total) {
		return (double)(0.98 * count)/total + 0.01;
	} else {
		return (za) ? 0.01 : 0.99;
	}
}

cv::Mat ChowLiuTree::buildTree(int root_word, list<info> &edges) {

	int q = root_word;
	cv::Mat cltree(4, edges.size()+1, CV_64F);

	cltree.at<double>(0, q) = q;
	cltree.at<double>(1, q) = P(q, true);
	cltree.at<double>(2, q) = P(q, true);
	cltree.at<double>(3, q) = P(q, true);
	//setting P(zq|zpq) to P(zq) gives the root node of the chow-liu 
	//independence from a parent node.

	//find all children and do the same
	vector<int> nextqs = extractChildren(edges, q);

	int pq = q;
	vector<int>::iterator nextq;
	for(nextq = nextqs.begin(); nextq != nextqs.end(); nextq++) {
		recAddToTree(cltree, *nextq, pq, edges);
	}

	return cltree;


}

void ChowLiuTree::recAddToTree(cv::Mat &cltree, int q, int pq, 
							   list<info>& remaining_edges) {

	cltree.at<double>(0, q) = pq;
	cltree.at<double>(1, q) = P(q, true);
	cltree.at<double>(2, q) = CP(q, true, pq, true);
	cltree.at<double>(3, q) = CP(q, true, pq, false);

	//find all children and do the same
	vector<int> nextqs = extractChildren(remaining_edges, q);

	pq = q;
	vector<int>::iterator nextq;
	for(nextq = nextqs.begin(); nextq != nextqs.end(); nextq++) {
		recAddToTree(cltree, *nextq, pq, remaining_edges);
	}
}

vector<int> ChowLiuTree::extractChildren(list<info> &remaining_edges, int q) {

	vector<int> children;
	list<info>::iterator edge = remaining_edges.begin();

	while(edge != remaining_edges.end()) {
		if(edge->word1 == q) {
			children.push_back(edge->word2);
			edge = remaining_edges.erase(edge);
			continue;
		}
		if(edge->word2 == q) {
			children.push_back(edge->word1);
			edge = remaining_edges.erase(edge);
			continue;
		}
		edge++;
	}

	return children;
}

bool ChowLiuTree::sortInfoScores(const info& first, const info& second) {
	return first.score > second.score;
}

double ChowLiuTree::calcMutInfo(int word1, int word2) {
	double accumulation = 0;

	double P00 = JP(word1, false, word2, false);
	if(P00) accumulation += P00 * log(P00 / (P(word1, false)*P(word2, false)));

	double P01 = JP(word1, false, word2, true);
	if(P01) accumulation += P01 * log(P01 / (P(word1, false)*P(word2, true)));

	double P10 = JP(word1, true, word2, false);
	if(P10) accumulation += P10 * log(P10 / (P(word1, true)*P(word2, false)));

	double P11 = JP(word1, true, word2, true);
	if(P11) accumulation += P11 * log(P11 / (P(word1, true)*P(word2, true)));

	return accumulation;
}

void ChowLiuTree::createBaseEdges(list<info>& edges, double infoThreshold) {

	int nWords = imgDescriptors[0].cols;
	info mutInfo;

	for(int word1 = 0; word1 < nWords; word1++) {
		for(int word2 = word1 + 1; word2 < nWords; word2++) {
			mutInfo.word1 = word1;
			mutInfo.word2 = word2;
			mutInfo.score = (float)calcMutInfo(word1, word2);
			if(mutInfo.score >= infoThreshold)
			edges.push_back(mutInfo);
		}
	}
	edges.sort(sortInfoScores);
}

bool ChowLiuTree::reduceEdgesToMinSpan(list<info>& edges) {

	map<int, int> groups; map<int, int>::iterator groupIt;
	for(int i = 0; i < imgDescriptors[0].cols; i++) groups[i] = i;
	int group1, group2;

	list<info>::iterator edge = edges.begin();
	while(edge != edges.end()) {
		if(groups[edge->word1] != groups[edge->word2]) {
			group1 = groups[edge->word1];
			group2 = groups[edge->word2];
			for(groupIt = groups.begin(); groupIt != groups.end(); groupIt++)
			if(groupIt->second == group2) groupIt->second = group1;
			edge++;
		} else {
			edge = edges.erase(edge);
		}
	}

	if(edges.size() != (unsigned int)imgDescriptors[0].cols - 1) {
		return false;
	} else {
		return true;
	}

}

}

