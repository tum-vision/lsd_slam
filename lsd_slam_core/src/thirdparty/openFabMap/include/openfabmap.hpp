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

#ifndef OPENFABMAP_H_
#define OPENFABMAP_H_

#include <vector>
#include <list>
#include <map>
#include <set>
#include <valarray>

#include <opencv2/opencv.hpp>

namespace of2 {


/*
	Return data format of a FABMAP compare call
*/
struct IMatch {

	IMatch() :
		queryIdx(-1), imgIdx(-1), likelihood(-DBL_MAX), match(-DBL_MAX) {
	}
	IMatch(int _queryIdx, int _imgIdx, double _likelihood, double _match) :
		queryIdx(_queryIdx), imgIdx(_imgIdx), likelihood(_likelihood), match(
				_match) {
	}

	int queryIdx;	//query index
	int imgIdx;		//test index 

	double likelihood;	//raw loglikelihood
	double match;		//normalised probability

	bool operator<(const IMatch& m) const {
		return match < m.match;
	}

};

/*
	Base FabMap class. Each FabMap method inherits from this class.
*/
class FabMap {
public:

	//FabMap options
	enum {
		MEAN_FIELD = 1,
		SAMPLED = 2,
		NAIVE_BAYES = 4,
		CHOW_LIU = 8,
		MOTION_MODEL = 16
	};

	FabMap(const cv::Mat& clTree, double PzGe, double PzGNe, int flags,
			int numSamples = 0);
	virtual ~FabMap();

	//methods to add training data for sampling method
	virtual void addTraining(const cv::Mat& queryImgDescriptor);
	virtual void addTraining(const std::vector<cv::Mat>& queryImgDescriptors);

	//methods to add to the test data
	virtual void add(const cv::Mat& queryImgDescriptor);
	virtual void add(const std::vector<cv::Mat>& queryImgDescriptors);

	//accessors
	const std::vector<cv::Mat>& getTrainingImgDescriptors() const;
	const std::vector<cv::Mat>& getTestImgDescriptors() const;

	//Main FabMap image comparison
	void compare(const cv::Mat& queryImgDescriptor,
			std::vector<IMatch>& matches, bool addQuery = false,
			const cv::Mat& mask = cv::Mat());
	void compare(const cv::Mat& queryImgDescriptor,
			const cv::Mat& testImgDescriptors, std::vector<IMatch>& matches,
			const cv::Mat& mask = cv::Mat());
	void compare(const cv::Mat& queryImgDescriptor,
			const std::vector<cv::Mat>& testImgDescriptors,
			std::vector<IMatch>& matches, const cv::Mat& mask = cv::Mat());
	void compare(const std::vector<cv::Mat>& queryImgDescriptors, std::vector<
			IMatch>& matches, bool addQuery = false, const cv::Mat& mask =
			cv::Mat());
	void compare(const std::vector<cv::Mat>& queryImgDescriptors,
			const std::vector<cv::Mat>& testImgDescriptors,
			std::vector<IMatch>& matches, const cv::Mat& mask = cv::Mat());

protected:

	void compareImgDescriptor(const cv::Mat& queryImgDescriptor,
			int queryIndex, const std::vector<cv::Mat>& testImgDescriptors,
			std::vector<IMatch>& matches);

	void addImgDescriptor(const cv::Mat& queryImgDescriptor);

	//the getLikelihoods method is overwritten for each different FabMap
	//method.
	virtual void getLikelihoods(const cv::Mat& queryImgDescriptor,
			const std::vector<cv::Mat>& testImgDescriptors,
			std::vector<IMatch>& matches);
	virtual double getNewPlaceLikelihood(const cv::Mat& queryImgDescriptor);
	
	//turn likelihoods into probabilities (also add in motion model if used)
	void normaliseDistribution(std::vector<IMatch>& matches);

	//Chow-Liu Tree
	int pq(int q);
	double Pzq(int q, bool zq);
	double PzqGzpq(int q, bool zq, bool zpq);
	
	//FAB-MAP Core
	double PzqGeq(bool zq, bool eq);
	double PeqGL(int q, bool Lzq, bool eq);
	double PzqGL(int q, bool zq, bool zpq, bool Lzq);
	double PzqGzpqL(int q, bool zq, bool zpq, bool Lzq);
	double (FabMap::*PzGL)(int q, bool zq, bool zpq, bool Lzq);

	//data
	cv::Mat clTree;
	std::vector<cv::Mat> trainingImgDescriptors;
	std::vector<cv::Mat> testImgDescriptors;
	std::vector<IMatch> priorMatches;

	//parameters
	double PzGe;
	double PzGNe;
	double Pnew;

	double mBias;
	double sFactor;

	int flags;
	int numSamples;

};

/*
	The original FAB-MAP algorithm, developed based on: 
	http://ijr.sagepub.com/content/27/6/647.short
*/
class FabMap1: public FabMap {
public:
	FabMap1(const cv::Mat& clTree, double PzGe, double PzGNe, int flags,
			int numSamples = 0);
	virtual ~FabMap1();
protected:

	//FabMap1 implementation of likelihood comparison
	void getLikelihoods(const cv::Mat& queryImgDescriptor, const std::vector<
			cv::Mat>& testImgDescriptors, std::vector<IMatch>& matches);
};

/*
	A computationally faster version of the original FAB-MAP algorithm. A look-
	up-table is used to precompute many of the reoccuring calculations
*/
class FabMapLUT: public FabMap {
public:
	FabMapLUT(const cv::Mat& clTree, double PzGe, double PzGNe,
			int flags, int numSamples = 0, int precision = 6);
	virtual ~FabMapLUT();
protected:

	//FabMap look-up-table implementation of the likelihood comparison
	void getLikelihoods(const cv::Mat& queryImgDescriptor, const std::vector<
			cv::Mat>& testImgDescriptors, std::vector<IMatch>& matches);

	//procomputed data
	int (*table)[8];

	//data precision
	int precision;
};

/*
	The Accelerated FAB-MAP algorithm, developed based on:
	http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=5613942
*/
class FabMapFBO: public FabMap {
public:
	FabMapFBO(const cv::Mat& clTree, double PzGe, double PzGNe, int flags,
			int numSamples = 0, double rejectionThreshold = 1e-8, double PsGd =
					1e-8, int bisectionStart = 512, int bisectionIts = 9);
	virtual ~FabMapFBO();

protected:

	//FabMap Fast Bail-out implementation of the likelihood comparison
	void getLikelihoods(const cv::Mat& queryImgDescriptor, const std::vector<
			cv::Mat>& testImgDescriptors, std::vector<IMatch>& matches);

	//stucture used to determine word comparison order
	struct WordStats {
		WordStats() :
			q(0), info(0), V(0), M(0) {
		}

		WordStats(int _q, double _info) :
			q(_q), info(_info), V(0), M(0) {
		}

		int q;
		double info;
		mutable double V;
		mutable double M;

		bool operator<(const WordStats& w) const {
			return info < w.info;
		}

	};

	//private fast bail-out necessary functions
	void setWordStatistics(const cv::Mat& queryImgDescriptor, std::multiset<
			WordStats>& wordData);
	double limitbisection(double v, double m);
	double bennettInequality(double v, double m, double delta);
	static bool compInfo(const WordStats& first, const WordStats& second);

	//parameters
	double PsGd;
	double rejectionThreshold;
	int bisectionStart;
	int bisectionIts;
};

/*
	The FAB-MAP2.0 algorithm, developed based on:
	http://ijr.sagepub.com/content/30/9/1100.abstract
*/
class FabMap2: public FabMap {
public:

	FabMap2(const cv::Mat& clTree, double PzGe, double PzGNe, int flags);
	virtual ~FabMap2();

	//FabMap2 builds the inverted index and requires an additional training/test
	//add function
	void addTraining(const cv::Mat& queryImgDescriptors) {
		FabMap::addTraining(queryImgDescriptors);
	}
	void addTraining(const std::vector<cv::Mat>& queryImgDescriptors);

	void add(const cv::Mat& queryImgDescriptors) {
		FabMap::add(queryImgDescriptors);
	}
	void add(const std::vector<cv::Mat>& queryImgDescriptors);

protected:

	//FabMap2 implementation of the likelihood comparison
	void getLikelihoods(const cv::Mat& queryImgDescriptor, const std::vector<
			cv::Mat>& testImgDescriptors, std::vector<IMatch>& matches);
	double getNewPlaceLikelihood(const cv::Mat& queryImgDescriptor);
	
	//the likelihood function using the inverted index
	void getIndexLikelihoods(const cv::Mat& queryImgDescriptor, std::vector<
			double>& defaults, std::map<int, std::vector<int> >& invertedMap,
			std::vector<IMatch>& matches);
	void addToIndex(const cv::Mat& queryImgDescriptor,
			std::vector<double>& defaults,
			std::map<int, std::vector<int> >& invertedMap);

	//data
	std::vector<double> d1, d2, d3, d4;
	std::vector<std::vector<int> > children;

	// TODO: inverted map a vector?

	std::vector<double> trainingDefaults;
	std::map<int, std::vector<int> > trainingInvertedMap;

	std::vector<double> testDefaults;
	std::map<int, std::vector<int> > testInvertedMap;

};
/*
	A Chow-Liu tree is required by FAB-MAP. The Chow-Liu tree provides an 
	estimate of the	full distribution of visual words using a minimum spanning 
	tree. The tree is generated through training data.
*/
class ChowLiuTree {
public:
	ChowLiuTree();
	virtual ~ChowLiuTree();

	//add data to the chow-liu tree before calling make
	void add(const cv::Mat& imgDescriptor);
	void add(const std::vector<cv::Mat>& imgDescriptors);

	const std::vector<cv::Mat>& getImgDescriptors() const;

	cv::Mat make(double infoThreshold = 0.0);

private:
	std::vector<cv::Mat> imgDescriptors;
	cv::Mat mergedImgDescriptors;

	typedef struct info {
		float score;
		short word1;
		short word2;
	} info;

	//probabilities extracted from mergedImgDescriptors
	double P(int a, bool za);
	double JP(int a, bool za, int b, bool zb); //a & b
	double CP(int a, bool za, int b, bool zb); // a | b

	//calculating mutual information of all edges
	void createBaseEdges(std::list<info>& edges, double infoThreshold);
	double calcMutInfo(int word1, int word2);
	static bool sortInfoScores(const info& first, const info& second);

	//selecting minimum spanning egdges with maximum information
	bool reduceEdgesToMinSpan(std::list<info>& edges);
	
	//building the tree sctructure
	cv::Mat buildTree(int root_word, std::list<info> &edges);
	void recAddToTree(cv::Mat &cltree, int q, int pq, 
		std::list<info> &remaining_edges);
	std::vector<int> extractChildren(std::list<info> &remaining_edges, int q);

};

/*
	A custom vocabulary training method based on:
	http://www.springerlink.com/content/d1h6j8x552532003/
*/
class BOWMSCTrainer: public cv::BOWTrainer {
public:
	BOWMSCTrainer(double clusterSize = 0.4);
	virtual ~BOWMSCTrainer();

	// Returns trained vocabulary (i.e. cluster centers).
	virtual cv::Mat cluster() const;
	virtual cv::Mat cluster(const cv::Mat& descriptors) const;

protected:

	double clusterSize;

};

}
#endif /* OPENFABMAP_H_ */
