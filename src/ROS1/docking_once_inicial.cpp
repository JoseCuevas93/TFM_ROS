#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <cmath> // Para cos() y sin()
#include "geometry_msgs/Twist.h" // Para utilizar geometry_msgs::Twist
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <cstdlib>
#include <ctime>
#include <string>

// Variables globales
int angulos_der = 0;
int angulos_izq = 0;
double x_extrem_izq = 0;
double y_extrem_izq = 0;
double x_extrem_der = 0;
double y_extrem_der = 0;

float dist_to_placa = 0.15;
float giro_inicial = 0;
float avance = 0;
float giro_final = 0;
float offset_lidar = 0.064; // 6.4 cm abajo del centro de giro

//bool move_to_origin_done = false;
bool move_to_lidar_done = false; 

// Variables para almacenar los valores de odometría
float inicial_x = 0.0, inicial_y = 0.0, inicial_w = 0.0;
float final_x = 0.0, final_y = 0.0, final_w = 0.0;

// Variable global para almacenar el tiempo 
std::string tiempo_inicial = "";
std::string tiempo_final = "";


// Función de callback global
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //ROS_INFO("entra lidar callback");
    
            if (move_to_lidar_done)
        {
            return;
            }

    const int max_angles_left = 180;
    const int max_angles_right = 180;
    std::vector<std::pair<int, float>> valid_ranges_left;
    std::vector<std::pair<int, float>> valid_ranges_right;
    std::vector<float> x_coords;
    std::vector<float> y_coords;

    float min_value = std::numeric_limits<float>::max();

    // Encontrar el valor más pequeño entre los primeros 180 valores
    for (int i = 0; i < max_angles_left; ++i) {
        if (msg->ranges[i] < min_value && msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
            min_value = msg->ranges[i];
        }
    }

    float last_valid_value_left = min_value;
    for (int i = 0; i < max_angles_left; ++i) {
        float current_value = msg->ranges[i];
        if (current_value >= msg->range_min && current_value <= msg->range_max) {
            if (current_value - last_valid_value_left <= 0.1) {
                int custom_angle = -i;
                valid_ranges_left.emplace_back(custom_angle, current_value);
                last_valid_value_left = current_value;
            }
        }
    }

    float last_valid_value_right = min_value;
    for (int i = max_angles_left; i < max_angles_left + max_angles_right; ++i) {
        float current_value = msg->ranges[i];
        if (current_value >= msg->range_min && current_value <= msg->range_max) {
            if (current_value - last_valid_value_right <= 0.1) {
                int custom_angle = 360 - i;
                valid_ranges_right.emplace_back(custom_angle, current_value);
                last_valid_value_right = current_value;
            }
        }
    }

    std::string output_file_path_txt = "/home/jose/ros/tfm_ws/src/tfm_simulation/lidar_data.txt";
    std::string output_file_path_csv = "/home/jose/ros/tfm_ws/src/tfm_simulation/lidar_data.csv";

    std::ofstream output_file_txt(output_file_path_txt);
    std::ofstream output_file_csv(output_file_path_csv);
    if (!output_file_txt.is_open() || !output_file_csv.is_open()) {
        ROS_ERROR("No se ha podido abrir el archivo para escribir los datos.");
        return;  // Eliminas el ros::shutdown() para seguir con la ejecución
    }

    output_file_csv << "Angle (degrees), Distance (meters), X, Y\n";

    output_file_txt << "Valores izquierda (0 a -179 grados):\n";
    output_file_txt << "Angle  Modulo  Coord_x Coord_y:\n";
    for (const auto& range : valid_ranges_left) {
        float angle_rad = range.first * msg->angle_increment + msg->angle_min;
        float x = range.second * sin(angle_rad);
        float y = range.second * cos(angle_rad);
        x_coords.push_back(x);
        y_coords.push_back(y);

        output_file_txt << range.first << " " << range.second << " " << x << " " << y << "\n";
        output_file_csv << range.first << ", " << range.second << ", " << x << ", " << y << "\n";
    }
    ////
x_extrem_izq = x_coords.back(); // Coordenada x del extrem esquerre de la placa

        // Escribimos los valores de derecha al archivo TXT y CSV (invertidos)
        output_file_txt << "\nValors dreta (1 a 180 graus):\n";
        output_file_txt << "Angle  Modul  Coord_x Coord_y:\n";
        for (auto it = valid_ranges_right.rbegin(); it != valid_ranges_right.rend(); ++it) {
            float angle_rad = it->first * msg->angle_increment + msg->angle_min;
            float x = it->second * sin(angle_rad); // Coordenada X en polars
            float y = it->second * cos(angle_rad); // Coordenada Y en polars
            x_coords.push_back(x);
            y_coords.push_back(y);

            // Escribir al archivo TXT
            output_file_txt << it->first << " " << it->second << " " << x << " " << y << "\n"; // Angle, distància i coordenades (x, y)

            // Escribir al archivo CSV
            output_file_csv << it->first << ", " << it->second << ", " << x << ", " << y << "\n"; // Angle, distància, coordenades (x, y)
        }

        x_extrem_der = x_coords.back(); // Coordenada x del extrem dret de la placa

        // Guardar el primer valor de la derecha y el último valor de la izquierda en variables globales
        if (!valid_ranges_right.empty()) {
            angulos_der = valid_ranges_right.front().first; // Primer valor de la derecha
        }

        if (!valid_ranges_left.empty()) {
            angulos_izq = valid_ranges_left.back().first; // Último valor de la izquierda
        }

        // Calcular la pendiente (m) y la intersección (b) del segmento utilizando la fórmula
        int n = x_coords.size();
        if (n > 1) {
            float sum_x = 0, sum_y = 0, sum_x2 = 0, sum_xy = 0;

            // Calcular las sumas necesarias
            for (int i = 0; i < n; ++i) {
                sum_x += x_coords[i];
                sum_y += y_coords[i];
                sum_x2 += x_coords[i] * x_coords[i];
                sum_xy += x_coords[i] * y_coords[i];
            }

            // Calcular la pendiente (m)
            float m = (sum_xy - (sum_x * sum_y) / n) / (sum_x2 - (sum_x * sum_x) / n);

            // Calcular la intersección (b)
            float b = (sum_y - m * sum_x) / n;

            y_extrem_izq = m * x_extrem_izq + b - offset_lidar;	// Calculo extremo con correccion offset
            y_extrem_der = m * x_extrem_der + b - offset_lidar;	// Calculo extremo con correccion offset

            // Calcular distància entre extrems
            float distancia_extrems = sqrt(pow(x_extrem_der - x_extrem_izq, 2) + pow(y_extrem_der - y_extrem_izq, 2));

            // Escriure resultats TXT
            output_file_txt << "Pendents (m), Intersecció (b), Distància entre extrems (m)\n";
            output_file_txt << m << ", " << b << ", " << distancia_extrems << "\n";
            output_file_txt << "x_extrem_izq: " << x_extrem_izq << "\n";
            output_file_txt << "y_extrem_izq: " << y_extrem_izq << "\n";
            output_file_txt << "x_extrem_der: " << x_extrem_der << "\n";
            output_file_txt << "y_extrem_der: " << y_extrem_der << "\n";

	    //Càlculs Trigonomètrics (dist_to_placa) fabs(numero);
	    float Pmx = (x_extrem_izq + x_extrem_der) / 2;	//Coordenada x del punto medio entre los dos extremos placa
	    float Pmy = (y_extrem_izq + y_extrem_der) / 2;	//Coordenada y del punto medio entre los dos extremos placa
	    
	    float m_inv = -1/m;	//Calcula la pendiente prependicula a la placa
	    float tecta = fabs(atan(m_inv));	//Angulo entre la pendiente inversa y eje x
	    
	    float Pdx = Pmx + (m/fabs(m)) * dist_to_placa * cos(tecta); //(m/fabs(m)) depen de la pendent de m se suma o se resta el coseno
	    float Pdy = Pmy - dist_to_placa * sin(fabs(tecta)); 	// Per a la coordenada y sempre se resta el sinus i
	    
	    float dist_orig_Pd = sqrt(Pdx*Pdx + Pdy*Pdy);		//Distancia Origen a punto Pd (distancia avance lineal)
	    avance = dist_orig_Pd;
	    
	    float alfa = fabs(atan(Pdy/Pdx));				//Angle Pd amb eix x
	    
	    if (Pdx >= 0){ //Pd esta en el quadrant 1
	    
	    	giro_inicial = - (1.5708  - alfa); //Gir sentit horari, el valor ha de ser negatiu -(90º - alfa)
	    	
	    		if (m >= 0){ //Pendent placa positiva
	    
	    			giro_final = M_PI  - alfa - tecta;	//180º - alfa - tecta
	    
	    		}
	    
	    		if (m < 0){ //Pendent placa negativa
	    
	    			
	    			giro_final = tecta - alfa;
	    
	    		}
	    	    
	    }
	    if (Pdx < 0){	//Pd esta en el quadrant 2
	    
	    	giro_inicial = M_PI_2  - alfa; 	//Gir sentit antihorari (positiu (90º - alfa))

	    		if (m >= 0){ //Pendent placa positiva
	    
	    			//giro_final = - fabs(tecta - alfa);
	    			giro_final = alfa - tecta;
	    
	    		}
	    
	    		if (m < 0){ //Pendent placa negativa
	    
	    			giro_final = -(M_PI  - alfa - tecta);	//180º - alfa - tecta
	    			
	    
	    		}	    	
	    	
	    }	    
	    
	// Escriure resultats de trigonometria en TXT
	output_file_txt << "Resultats càlculs trigonomètrics:\n";
	output_file_txt << "giro_inicial (radians): " << giro_inicial << "\n";
	output_file_txt << "giro_inicial (graus): " << (giro_inicial * 180.0 / M_PI) << "\n";
	output_file_txt << "avance: " << avance << "\n"; 
	output_file_txt << "giro_final (radians): " << giro_final << "\n";
	output_file_txt << "giro_final (graus): " << (giro_final * 180.0 / M_PI) << "\n";  
	// Alfa
	output_file_txt << "alfa (radians): " << alfa << "\n";
	output_file_txt << "alfa (graus): " << (alfa * 180.0 / M_PI) << "\n";

	// Tecta
	output_file_txt << "tecta (radians): " << tecta << "\n";
	output_file_txt << "tecta (graus): " << (tecta * 180.0 / M_PI) << "\n";

	// Pdx
	output_file_txt << "Pdx: " << Pdx << "\n";

	// Pdy
	output_file_txt << "Pdy: " << Pdy << "\n";  
	
	output_file_txt << "M_PI_2: " << M_PI_2 << "\n";  
	output_file_txt << "M_PI: " << M_PI << "\n";  

            // Escribir resultados CSV
            output_file_csv << "\nPendents (m), Intersecció (b), Distància entre extrems (m), x_extrem_izq, y_extrem_izq, x_extrem_der, y_extrem_der\n";
            output_file_csv << m << ", " << b << ", " << distancia_extrems << ", " << x_extrem_izq << ", " << y_extrem_izq << ", " << x_extrem_der << ", " << y_extrem_der << "\n";
     
        ///////////
    }

    output_file_txt.close();
    output_file_csv.close();

    ROS_INFO("Datos del LIDAR guardados en los archivos: %s y %s", output_file_path_txt.c_str(), output_file_path_csv.c_str());
    move_to_lidar_done = true;
    
}

void lidarToFile() {
    ros::NodeHandle nh;

    // Crear el suscriptor
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Rate rate(10); // Frecuencia de ejecución (10 Hz)
    move_to_lidar_done = false;
    
        while (ros::ok() && !move_to_lidar_done)
    {
        ros::spinOnce(); // Procesa callbacks
        rate.sleep();    // Mantiene la frecuencia
    }

}

void mover_robot() {
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    geometry_msgs::Twist twist;

    // Constantes de velocidad (ajustables según el robot)
    float angular_velocity = 0.2; // radianes/s
    float linear_velocity = 0.1;  // metros/s
    
    ros::Duration(0.5).sleep(); // Espera 0.5 segundos antes de iniciar el movimiento

    // 1. Gira lo que valga giro_inicial
    twist.angular.z = (giro_inicial < 0 ? -angular_velocity : angular_velocity);
    twist.linear.x = 0.0;
    cmd_vel_pub.publish(twist);

    // Calcula el tiempo necesario para completar el giro
    double giro_time = fabs(giro_inicial) / angular_velocity;
    ros::Duration(giro_time).sleep(); // Mantén el giro durante el tiempo necesario

    // Para el giro
    twist.angular.z = 0.0;
    cmd_vel_pub.publish(twist);
    
    ros::Duration(0.5).sleep(); // Espera 0.5 segundos

    // 2. Avanza en línea recta lo que valga avance
    twist.linear.x = (avance < 0 ? -linear_velocity : linear_velocity);
    twist.angular.z = 0.0;  // Asegúrate de que no haya rotación mientras avanza
    cmd_vel_pub.publish(twist);

    // Calcula el tiempo necesario para avanzar
    double avance_time = fabs(avance) / linear_velocity;
    ros::Duration(avance_time).sleep(); // Mantén el avance durante el tiempo necesario

    // Para el avance
    twist.linear.x = 0.0;
    cmd_vel_pub.publish(twist);

    ros::Duration(1).sleep(); // Espera 1 segundos

    // 3. Gira lo que valga giro_final
    twist.angular.z = (giro_final < 0 ? -angular_velocity : angular_velocity);
    cmd_vel_pub.publish(twist);

    // Calcula el tiempo necesario para completar el giro
    double giro_final_time = fabs(giro_final) / angular_velocity;
    ros::Duration(giro_final_time).sleep(); // Mantén el giro durante el tiempo necesario

    // Para el giro
    twist.angular.z = 0.0;
    cmd_vel_pub.publish(twist);
    
    ROS_INFO("Finalizacion DOCKING.");
}

// Función para convertir un cuaternión a yaw
double getYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  // Convierte cuaternión a ángulos de Euler
    return yaw;  // Devuelve el yaw (rotación alrededor del eje Z)
}

// Función para calcular la odometría
void odom_inicio(float& x, float& y, float& yaw) {

    ros::NodeHandle nh;
    bool is_processed = false;

    // Callback que se ejecutará al recibir el mensaje
    auto odomCallback = [&x, &y, &yaw, &is_processed](const nav_msgs::Odometry::ConstPtr& msg) {
        if (!is_processed) {
            x = msg->pose.pose.position.x;
            y = msg->pose.pose.position.y;

            // Calcular el yaw a partir del cuaternión de orientación
            yaw = getYawFromQuaternion(msg->pose.pose.orientation);

            ROS_INFO("Odom values: position.x=%.2f, position.y=%.2f, yaw=%.4f", x, y, yaw);
            is_processed = true; // Marcar como procesado
        }
    };

    // Crear el suscriptor al tópico de odometría
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, odomCallback);

    // Esperar hasta que se procese un mensaje
    ros::Rate rate(10); // Frecuencia en Hz
    while (ros::ok() && !is_processed) {
        ros::spinOnce(); // Procesar mensajes
        rate.sleep();
    }

    // Ruta del archivo
    std::string file_path = "/home/jose/ros/tfm_ws/src/tfm_simulation/resultados.txt";

    // Abrir el archivo en modo de añadir datos
    std::ofstream file(file_path, std::ios::app);
    if (file.is_open()) {
        file << ", los valores de odometría inicial son: "
             << "x=" << x << ", y=" << y << ", yaw=" << yaw << "\n";

        // Obtener el tiempo actual
        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);
        char time_str[100];
        std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&current_time));
        
        tiempo_inicial = time_str; // Guarda tiempo en varaible global

        //file << "Tiempo: " << time_str << "\n";
        file.close();
        ROS_INFO("Datos escritos en el archivo: %s", file_path.c_str());
        
    } else {
        ROS_ERROR("No se pudo abrir el archivo: %s", file_path.c_str());
    }
}

// Función para calcular la odometría
void odom_final(float& x, float& y, float& yaw) {

    ros::NodeHandle nh;
    bool is_processed = false;

    // Callback que se ejecutará al recibir el mensaje
    auto odomCallback = [&x, &y, &yaw, &is_processed](const nav_msgs::Odometry::ConstPtr& msg) {
        if (!is_processed) {
            x = msg->pose.pose.position.x;
            y = msg->pose.pose.position.y;

            // Calcular el yaw a partir del cuaternión de orientación
            yaw = getYawFromQuaternion(msg->pose.pose.orientation);

            ROS_INFO("Odom values: position.x=%.2f, position.y=%.2f, yaw=%.4f", x, y, yaw);
            is_processed = true; // Marcar como procesado
        }
    };

    // Crear el suscriptor al tópico de odometría
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, odomCallback);

    // Esperar hasta que se procese un mensaje
    ros::Rate rate(10); // Frecuencia en Hz
    while (ros::ok() && !is_processed) {
        ros::spinOnce(); // Procesar mensajes
        rate.sleep();
    }

    // Ruta del archivo
    std::string file_path = "/home/jose/ros/tfm_ws/src/tfm_simulation/resultados.txt";

    // Abrir el archivo en modo de añadir datos
    std::ofstream file(file_path, std::ios::app);
    if (file.is_open()) {
        file << ", los valores de odometría final son: "
             << "x=" << x << ", y=" << y << ", yaw=" << yaw << "\n";

        // Obtener el tiempo actual
        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);
        char time_str[100];
        std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&current_time));

        tiempo_final = time_str; // Guarda tiempo en varaible global
        
        //file << "Tiempo: " << time_str << "\n";
        file.close();
        ROS_INFO("Datos escritos en el archivo: %s", file_path.c_str());
        
    } else {
        ROS_ERROR("No se pudo abrir el archivo: %s", file_path.c_str());
    }
}

// Función para convertir un string HH:MM:SS a segundos desde las 00:00:00
int convertirATiempoEnSegundos(const std::string& tiempo) {
    std::tm t = {};
    std::istringstream ss(tiempo);
    ss >> std::get_time(&t, "%H:%M:%S");
    if (ss.fail()) {
        throw std::invalid_argument("Formato de tiempo no válido: " + tiempo);
    }
    return t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
}

// Función para convertir radianes a grados
float radiansToDegrees(float radians) {
    return radians * (180.0 / M_PI);
}

// Función que calcula y escribe los resultados
void calcularYEscribir() {
    // Definir la ruta del archivo como variable local
    std::string file_path = "/home/jose/ros/tfm_ws/src/tfm_simulation/resultados.txt";

    // Punto de referencia
    float ref_x = -0.35, ref_y = 0.0;
    float ref_w = M_PI;

    // Calcular distancia euclidiana al punto de referencia
    float distancia = std::sqrt(std::pow(final_x - ref_x, 2) + std::pow(final_y - ref_y, 2));

    // Calcular la diferencia angular y convertir a grados
    float diferencia_w = abs(radiansToDegrees(abs(final_w) - ref_w));

    // Calcular la diferencia de tiempo en segundos
    int diferencia_tiempo = 0;
    try {
        if (!tiempo_inicial.empty() && !tiempo_final.empty()) {
            int tiempo_inicial_segundos = convertirATiempoEnSegundos(tiempo_inicial);
            int tiempo_final_segundos = convertirATiempoEnSegundos(tiempo_final);
            diferencia_tiempo = tiempo_final_segundos - tiempo_inicial_segundos;
        }
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error al calcular la diferencia de tiempo: " << e.what() << std::endl;
    }

    // Escribir los resultados en el archivo
    std::ofstream file(file_path, std::ios::app); // Abrir en modo de append
    if (file.is_open()) {
        file << "Distancia al punto (-0.35, 0): " << distancia << " metros\n";
        file << "Diferencia angular con PI: " << diferencia_w << " grados\n";
        file << "Diferencia de tiempo: " << diferencia_tiempo << " segundos\n";
        file.close();
        std::cout << "Resultados guardados en " << file_path << std::endl;
    } else {
        std::cerr << "No se pudo abrir el archivo " << file_path << std::endl;
    }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "docking_loop");
    ros::start();
    ros::Rate rate(10); // Frecuencia del bucle

    

        // Calcular odometría inicial
        ROS_INFO("Calcular odometría inicial");
        odom_inicio(inicial_x, inicial_y, inicial_w);

        ROS_INFO("Pasando a datos lidar");
        ros::Duration(0.5).sleep();

        // Procesar datos del lidar y calcular movimientos Docking
        lidarToFile();

        ROS_INFO("Pasando a mover robot");
        ros::Duration(0.5).sleep();

        // Mover el robot
        mover_robot();

        ros::Duration(0.5).sleep();

        // Calcular odometría final
        ROS_INFO("Calcular odometria final");
        odom_final(final_x, final_y, final_w);

        // Escribir resultados
        ROS_INFO("Escribir resultados en archivo");
        calcularYEscribir();
         
    // Realiza shutdown del nodo
    ros::shutdown();

    return 0;
}


