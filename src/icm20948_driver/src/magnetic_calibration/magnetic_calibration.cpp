//
// Created by lemezoth on 29/02/24.
//

#include "icm20948_driver/magnetic_calibration/magnetic_calibration.h"

#include <rosbag2_cpp/serialization_format_converter_factory.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include <iostream>
#include <vector>

#include "icm20948_driver/magnetic_calibration/magnetic_paving.h"

#include <vtkSmartPointer.h>
#include <vtkCamera.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>

#include <vtkChartXYZ.h>
#include <vtkContext3D.h>
#include <vtkContextScene.h>
#include <vtkContextView.h>
#include <vtkFloatArray.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPen.h>
#include <vtkPlotLine3D.h>
#include <vtkTable.h>
#include <vtkGlyph3D.h>
#include <vtkAppendPolyData.h>

#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

#include <vtkParametricEllipsoid.h>
#include <vtkParametricFunctionSource.h>

#include <vtkNamedColors.h>
#include <vtkProperty.h>

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>


MagneticCalibration::MagneticCalibration()
        : Node("magnetic_calibration"){

    init_parameters();
    read_raw_data();
    regularize_data();
    compute_ellipsoid();
    view_data();

    exit(EXIT_SUCCESS);
}

void MagneticCalibration::init_parameters() {
    this->declare_parameter<std::string>("bag_path", bag_path_);
    bag_path_ = this->get_parameter_or("bag_path", bag_path_);
    RCLCPP_INFO(this->get_logger(), "[MagneticCalibration] bag_path: %s", bag_path_.c_str());

    this->declare_parameter<std::string>("topic_raw_imu_name", topic_raw_imu_name_);
    topic_raw_imu_name_ = this->get_parameter_or("topic_raw_imu_name", topic_raw_imu_name_);

    this->declare_parameter<double>("bisect_limit_width", bisect_limit_width_);
    bisect_limit_width_ = this->get_parameter_or("bisect_limit_width", bisect_limit_width_);

    this->declare_parameter<int>("bisect_limit_nb_data", bisect_limit_nb_data_);
    bisect_limit_nb_data_ = this->get_parameter_or("bisect_limit_nb_data", bisect_limit_nb_data_);

}

void MagneticCalibration::read_raw_data(){
    // Open the rosbag
    rosbag2_storage::StorageOptions storage_options({bag_path_, "mcap"});;
    rosbag2_cpp::ConverterOptions converter_options({"cdr", "cdr"});
    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options, converter_options);

    // Filter to imu raw data
    rosbag2_storage::StorageFilter storage_filter;
    storage_filter.topics.push_back(topic_raw_imu_name_);
    reader.set_filter(storage_filter);

    // Message deserialized
    rclcpp::Serialization<icm20948_driver::msg::RawData> serialization;
    icm20948_driver::msg::RawData msg;

    while (reader.has_next()){
        auto msg_serialized = reader.read_next();

        rclcpp::SerializedMessage extracted_serialized_msg(*msg_serialized->serialized_data);
        serialization.deserialize_message(
                &extracted_serialized_msg, &msg);
        raw_data_list_.push_back(msg);
    }

    // Extract magnetometer data
    magnetometer_data_.reserve(raw_data_list_.size());
    for (auto & i : raw_data_list_)
        magnetometer_data_.push_back({i.mag.x, i.mag.y, i.mag.z});

    cout << "Load " << magnetometer_data_.size() << " values" << endl;
}

void MagneticCalibration::regularize_data(){
    // Regularize the data
    MagneticPaving magnetic_paving(magnetometer_data_, 30, 1.0);
    magnetic_paving.process_data(magnetometer_data_regularized_);

    // Display min max
    magnetic_paving.cout_bounds();

    cout << "Regularized " << magnetometer_data_regularized_.size() << " values" << endl;
}

void MagneticCalibration::compute_ellipsoid() {
    // Compute the ellipsoid
    MatrixXd data(magnetometer_data_regularized_.size(), 3);
    for (size_t i = 0; i < magnetometer_data_regularized_.size(); ++i)
        for (size_t j = 0; j < 3; ++j)
            data(i, j) = magnetometer_data_regularized_[i][j];

    // http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit

    // Create D matrix
    MatrixXd D(9, data.rows());
    for (size_t i = 0; i < data.rows(); ++i) {
        D(0, i) = data(i, 0) * data(i, 0);
        D(1, i) = data(i, 1) * data(i, 1);
        D(2, i) = data(i, 2) * data(i, 2);
        D(3, i) = 2 * data(i, 0) * data(i, 1);
        D(4, i) = 2 * data(i, 0) * data(i, 2);
        D(5, i) = 2 * data(i, 1) * data(i, 2);
        D(6, i) = 2 * data(i, 0);
        D(7, i) = 2 * data(i, 1);
        D(8, i) = 2 * data(i, 2);
    }

    // Compute the pseudo-inverse of D with JacobiSVD
    VectorXd v = (D*D.transpose()).jacobiSvd(ComputeThinU | ComputeThinV).solve(D*VectorXd::Ones(data.rows()));

    // Construct the matrix A
    Matrix4d A;
    A << v[0], v[3], v[4], v[6],
            v[3], v[1], v[5], v[7],
            v[4], v[5], v[2], v[8],
            v[6], v[7], v[8], -1.;

    // Compute the center of the ellipsoid
    center_ = (-A.topLeftCorner<3, 3>()).inverse()*Vector3d(v[6], v[7], v[8]);

    // Compute the transformation matrix T
    Matrix4d T = Matrix4d::Identity();
    T.row(3).head(3) = center_;

    // Compute the rotated ellipsoid
    R_ = T * A * T.transpose();

    // Extract eigenvalues and eigenvectors of the rotated ellipsoid
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(R_.topLeftCorner<3, 3>() / -R_(3, 3));
    Vector3d evals_ = eigen_solver.eigenvalues();
    Matrix3d evecs_ = eigen_solver.eigenvectors();

    // radii = np.sqrt(1. / evals)
    Vector3d radii = (1.0 / evals_.array().sqrt()).matrix();
    double r = std::cbrt(radii.prod());
    Matrix3d D1 = r * radii.asDiagonal().inverse();
    Matrix3d TR = evecs_ * D1 * evecs_.transpose();

    cout << "Ellipsoid center: " << endl << center_.transpose() << endl;
//    cout << "Ellipsoid radii: " << evals_.transpose() << endl;
//    cout << "Ellipsoid evecs: " << evecs_ << endl;
    cout << "Ellipsoid TR: " << endl << TR << endl;
}

vtkSmartPointer<vtkActor> MagneticCalibration::generate_point_cloud(vector<array<double, 3>> &pts_data, const string &color, const double &radius){
    vtkNew<vtkNamedColors> colors;

    // Create a vtkPoints object to store the points
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    // Add points to vtkPoints object
    for (const auto& point : pts_data) {
        points->InsertNextPoint(point.data());
    }

    // Create a polydata to hold the points
    vtkSmartPointer<vtkPolyData> polydata_points = vtkSmartPointer<vtkPolyData>::New();
    polydata_points->SetPoints(points);

    // Create sphere source to represent the points
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(radius); // Set sphere radius
    sphereSource->SetPhiResolution(10);
    sphereSource->SetThetaResolution(10);

    // Create glyph filter
    vtkSmartPointer<vtkGlyph3D> glyphFilter = vtkSmartPointer<vtkGlyph3D>::New();
    glyphFilter->SetInputData(polydata_points);
    glyphFilter->SetSourceConnection(sphereSource->GetOutputPort());
    glyphFilter->SetScaleFactor(1.0); // Set scale factor if needed

    // Create mapper and actor for combined data
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());

    return actor;
}

vtkSmartPointer<vtkMatrix4x4> eigenToVtkMatrix(const Eigen::Matrix4d& eigenMatrix) {
    vtkSmartPointer<vtkMatrix4x4> vtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            vtkMatrix->SetElement(i, j, eigenMatrix(i, j));
        }
    }
    return vtkMatrix;
}

vtkSmartPointer<vtkActor> MagneticCalibration::generate_ellipsoid(const string &color){
    vtkNew<vtkNamedColors> colors;

    // Create a parametric ellipsoid
    vtkSmartPointer<vtkParametricEllipsoid> ellipsoid = vtkSmartPointer<vtkParametricEllipsoid>::New();
    ellipsoid->SetXRadius(evals_[0]);
    ellipsoid->SetYRadius(evals_[1]);
    ellipsoid->SetZRadius(evals_[2]);

    // Transform the ellipsoid
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(center_[0], center_[1], center_[2]);
    transform->Concatenate(eigenToVtkMatrix(R_));

    // Create a parametric function source and set the ellipsoid as the function
    vtkSmartPointer<vtkParametricFunctionSource> ellipsoidSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    ellipsoidSource->SetParametricFunction(ellipsoid);
    ellipsoidSource->Update();

    // Create a transform filter
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputData(ellipsoidSource->GetOutput());
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    // Create mapper and actor for combined data
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());

    return actor;
}

void MagneticCalibration::view_data(){

    vtkSmartPointer<vtkActor> actor_regularized = generate_point_cloud(magnetometer_data_regularized_, "Red", 1.0);
    vtkSmartPointer<vtkActor> actor_raw = generate_point_cloud(magnetometer_data_, "Black", 0.3);
    vtkSmartPointer<vtkActor> actor_ellipsoid = generate_ellipsoid("Green");


    // Create renderer, render window, and interactor
    vtkSmartPointer<vtkCamera> sharedCamera = vtkSmartPointer<vtkCamera>::New();
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetActiveCamera(sharedCamera);
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetSize(1000, 1000);
    renderWindow->SetWindowName("Magnetic Calibration");
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Add actor to the renderer
    renderer->AddActor(actor_regularized);
    renderer->AddActor(actor_raw);
    renderer->AddActor(actor_ellipsoid);

    renderer->SetBackground(0.1, 0.2, 0.4); // Set background color

    // Create axes
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(100.0, 100.0, 100.0);
    renderer->AddActor(axes);

    renderer->ResetCamera();
    sharedCamera->SetFocalPoint(0, 0, 0);

    // Start the interactor
    renderWindow->Render();
    renderWindowInteractor->Start();  // Start the event loop
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MagneticCalibration>());
    rclcpp::shutdown();

    return 0;
}
