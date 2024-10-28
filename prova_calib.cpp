#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

int main() {
    // Parametri del checkerboard
    cv::Size checkerboardSize(9, 6); // Numero di intersezioni: (colonne, righe)
    float squareSize = 0.025; // Dimensione del quadrato in metri

    // Crea un array di punti 3D
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < checkerboardSize.height; i++) {
        for (int j = 0; j < checkerboardSize.width; j++) {
            objp.emplace_back(j * squareSize, i * squareSize, 0);
        }
    }

    // Inizializza le liste per i punti
    std::vector<std::vector<cv::Point3f>> objPoints; // Punti 3D nel mondo
    std::vector<std::vector<cv::Point2f>> imgPointsRGB; // Punti 2D nell'immagine RGB
    std::vector<std::vector<cv::Point2f>> imgPointsIR; // Punti 2D nell'immagine IR

    // Acquisizione delle immagini (questo è solo un esempio; carica le tue immagini qui)
    std::vector<cv::Mat> imagesRGB; // Le tue immagini RGB
    std::vector<cv::Mat> imagesDepth; // Le tue immagini Depth

    for(int i=0; i<n; i++)
        {
            std::string filename="image_"+std::to_string(i)+".png";
            cv::Mat image=cv:imread(filename);
            if (img.empty()) {
                std::cerr << "Errore nel caricamento dell'immagine RGB: " << path << std::endl;
                continue;
             }
            imagesRGB.push_back(image);
        }



    for(int i=0; i<n; i++)
        {
            std::string filename="depth_"+std::to_string(i)+".png";
            cv::Mat image=cv:imread(filename);
            if (img.empty()) {
                std::cerr << "Errore nel caricamento dell'immagine Depth: " << path << std::endl;
                continue;
             }
            imagesDepth.push_back(image);
        }

    for (size_t i = 0; i < imagesRGB.size(); i++) {
        std::vector<cv::Point2f> cornersRGB, cornersIR;

        bool foundRGB = cv::findChessboardCorners(imagesRGB[i], checkerboardSize, cornersRGB);
        bool foundIR = cv::findChessboardCorners(imagesIR[i], checkerboardSize, cornersIR);

        if (foundRGB && foundIR) {
            objPoints.push_back(objp);
            imgPointsRGB.push_back(cornersRGB);
            imgPointsIR.push_back(cornersIR);
        }
    }

    // Calibrazione stereoscopica
    cv::Mat cameraMatrixRGB = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffsRGB = cv::Mat::zeros(8, 1, CV_64F);
    cv::Mat cameraMatrixIR = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffsIR = cv::Mat::zeros(8, 1, CV_64F);
    cv::Mat R, T, E, F;

    double rms = cv::stereoCalibrate(
        objPoints, imgPointsRGB, imgPointsIR,
        cameraMatrixRGB, distCoeffsRGB,
        cameraMatrixIR, distCoeffsIR,
        imagesRGB[0].size(), // Dimensione dell'immagine
        R, T, E, F,
        cv::CALIB_FIX_INTRINSIC
    );

    std::cout << "RMS error: " << rms << std::endl;
    std::cout << "Camera Matrix RGB: " << cameraMatrixRGB << std::endl;
    std::cout << "Camera Matrix IR: " << cameraMatrixIR << std::endl;
    std::cout << "Rotation Matrix: " << R << std::endl;
    std::cout << "Translation Vector: " << T << std::endl;

    // Raddrizzamento delle immagini
    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect roi1, roi2;

    cv::stereoRectify(
        cameraMatrixRGB, distCoeffsRGB,
        cameraMatrixIR, distCoeffsIR,
        imagesRGB[0].size(), R, T,
        R1, R2, P1, P2, Q,
        0, -1, roi1, roi2
    );

    // Qui puoi utilizzare P1, P2 e Q per la mappatura di profondità

    return 0;
}
