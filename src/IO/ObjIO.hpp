#include "FileIO.hpp"

class ObjIO : public FileIO {
public:
	void RealLoadFromFile(const std::string &filename, VectorXd &vertices, MatrixXi &topo) const override;
	void RealSaveToFile(const std::string &filename, const MatrixXd &vertices, const MatrixXi &topo) const override;
};