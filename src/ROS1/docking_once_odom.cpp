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

// Variables globales para almacenar la odometría
float x = 0.0, y = 0.0, yaw = 0.0;

// Variable global para almacenar el tiempo 
std::string tiempo_inicial = "";
std::string tiempo_final = "";


// Función de callback global
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (move_to_lidar_done) {
        return;
    }

    int num_beams = msg->ranges.size();
    int half_beams = num_beams / 2;

    std::vector<std::pair<double, float>> valid_ranges_left;
    std::vector<std::pair<double, float>> valid_ranges_right;
    std::vector<float> x_coords;
    std::vector<float> y_coords;

    float min_value = std::numeric_limits<float>::max();

    // Encontrar el valor más pequeño en la primera mitad de los haces (izquierda)
    for (int i = 0; i < half_beams; ++i) {
        if (msg->ranges[i] < min_value && msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
            min_value = msg->ranges[i];
        }
    }

    float last_valid_value_left = min_value;
    for (int i = 0; i < half_beams; ++i) {
        float current_value = msg->ranges[i];
        if (current_value >= msg->range_min && current_value <= msg->range_max) {
            if (current_value - last_valid_value_left <= 0.1) {
                double custom_angle = -i * (360.0 / num_beams);  // Mantener como double
                valid_ranges_left.emplace_back(custom_angle, current_value);
                last_valid_value_left = current_value;
            }
        }
    }

    float last_valid_value_right = min_value;
    for (int i = half_beams; i < num_beams; ++i) {
        float current_value = msg->ranges[i];
        if (current_value >= msg->range_min && current_value <= msg->range_max) {
            if (current_value - last_valid_value_right <= 0.1) {
                double custom_angle = 360.0 - i * (360.0 / num_beams);  // Mantener como double
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
        //float angle_rad = range.first * msg->angle_increment + msg->angle_min;
        float angle_rad = range.first * (M_PI / 180.0); // Conversión corregida
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
            //float angle_rad = it->first * msg->angle_increment + msg->angle_min;
            float angle_rad = it->first * (M_PI / 180.0); // Conversión corregida
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


// Función para convertir un cuaternión a yaw
double getYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

// Callback para la odometría (se actualiza cada vez que llega un mensaje de /odom)
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    yaw = getYawFromQuaternion(msg->pose.pose.orientation);

    ROS_INFO("Odom: x=%.2f, y=%.2f, yaw=%.4f", x, y, yaw);
}

// Función para registrar la odometría inicial
void odom_inicio() {
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_callback);

    ros::Rate rate(10); // 10 Hz
    bool is_processed = false;

    // Esperar hasta que se reciba un mensaje de odometría
    while (ros::ok() && !is_processed) {
        ros::spinOnce(); // Procesar mensaje de odometría
        if (x != 0.0 || y != 0.0 || yaw != 0.0) {
            is_processed = true;
        }
        rate.sleep();
    }

    // Guardar los valores iniciales en un archivo
    std::string file_path = "/home/jose/ros/tfm_ws/src/tfm_simulation/resultados.txt";
    std::ofstream file(file_path, std::ios::app);
    if (file.is_open()) {
        file << "Odometría inicial: x=" << x << ", y=" << y << ", yaw=" << yaw << "\n";

        // Obtener la hora actual
        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);
        char time_str[100];
        std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&current_time));

        tiempo_inicial = time_str;
        file << "Tiempo: " << time_str << "\n";
        file.close();
        ROS_INFO("Datos escritos en el archivo: %s", file_path.c_str());
    } else {
        ROS_ERROR("No se pudo abrir el archivo: %s", file_path.c_str());
    }
}

void odom_final() {
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_callback);

    ros::Rate rate(10); // 10 Hz
    bool is_processed = false;

    // Esperar hasta que se reciba un mensaje de odometría
    while (ros::ok() && !is_processed) {
        ros::spinOnce(); // Procesar mensaje de odometría
        if (x != 0.0 || y != 0.0 || yaw != 0.0) {
            is_processed = true;
        }
        rate.sleep();
    }

    // Ruta del archivo
    std::string file_path = "/home/jose/ros/tfm_ws/src/tfm_simulation/resultados.txt";

    // Guardar los valores finales en un archivo
    std::ofstream file(file_path, std::ios::app);
    if (file.is_open()) {
        file << "Odometría final: x=" << x << ", y=" << y << ", yaw=" << yaw << "\n";

        // Obtener el tiempo actual
        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);
        char time_str[100];
        std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&current_time));

        tiempo_final = time_str;
        file << "Tiempo final: " << time_str << "\n";
        file.close();
        ROS_INFO("Datos de odometría final escritos en el archivo: %s", file_path.c_str());
    } else {
        ROS_ERROR("No se pudo abrir el archivo: %s", file_path.c_str());
    }
}

void mover_robot() {
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_callback);
    
    geometry_msgs::Twist twist;
    float K = 0.5; // Ganancia del control proporcional

    ROS_INFO("Pulsa botón para enviar Twist 1");
    getchar();
    ros::Duration(0.5).sleep();

    // 1. Gira lo que valga giro_inicial con control proporcional
    float yaw_inicial = yaw;
    float error = 0.0;
    uint cnt = 0;
    bool converged = false;

    while (!converged && ros::ok()) {
        ros::spinOnce();
        float yaw_actual = yaw - yaw_inicial;
        error = giro_inicial - yaw_actual;
        twist.angular.z = K * error;
        twist.linear.x = 0.0;
        cmd_vel_pub.publish(twist);
        ros::Duration(0.1).sleep();

        if (fabs(error) < 0.01) {
            cnt++;
            if (cnt > 10) converged = true;
        } else {
            cnt = 0;
        }
    }

    twist.angular.z = 0.0;
    cmd_vel_pub.publish(twist);

    ROS_INFO("Pulsa botón para enviar Twist 2");
    getchar();
    ros::Duration(0.5).sleep();

    // 2. Avanza en línea recta con control proporcional usando distancia euclidiana
    float x_inicial = x;
    float y_inicial = y;
    cnt = 0;
    converged = false;

    while (!converged && ros::ok()) {
        ros::spinOnce();
        float distancia_actual = sqrt(pow(x - x_inicial, 2) + pow(y - y_inicial, 2));
        float error_dist = avance - distancia_actual;
        twist.linear.x = K * error_dist;
        twist.angular.z = 0.0;
        cmd_vel_pub.publish(twist);
        ros::Duration(0.1).sleep();

        if (fabs(error_dist) < 0.01) {
            cnt++;
            if (cnt > 10) converged = true;
        } else {
            cnt = 0;
        }
    }

    twist.linear.x = 0.0;
    cmd_vel_pub.publish(twist);

    ROS_INFO("Pulsa botón para enviar Twist 3");
    getchar();
    ros::Duration(1).sleep();

    // 3. Gira lo que valga giro_final con control proporcional
    yaw_inicial = yaw;
    error = giro_final;
    cnt = 0;
    converged = false;

    while (!converged && ros::ok()) {
        ros::spinOnce();
        float yaw_actual = yaw - yaw_inicial;
        error = giro_final - yaw_actual;
        twist.angular.z = K * error;
        twist.linear.x = 0.0;
        cmd_vel_pub.publish(twist);
        ros::Duration(0.1).sleep();

        if (fabs(error) < 0.01) {
            cnt++;
            if (cnt > 10) converged = true;
        } else {
            cnt = 0;
        }
    }

    twist.angular.z = 0.0;
    cmd_vel_pub.publish(twist);
    ROS_INFO("Finalización DOCKING.");
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

    ros::init(argc, argv, "docking_loop");  // Inicializa el nodo ROS
    ros::NodeHandle nh;
    ros::Rate rate(10); // Frecuencia de 10 Hz

    // Calcular odometría inicial
    ROS_INFO("Calculando odometría inicial...");
    odom_inicio();
    ros::Duration(0.5).sleep();

    // Procesar datos del LIDAR
    ROS_INFO("Procesando datos del LIDAR...");
    lidarToFile();
    ros::Duration(0.5).sleep();

    // Mover el robot
    ROS_INFO("Moviendo el robot...");
    mover_robot();
    ros::Duration(0.5).sleep();

    // Calcular odometría final
    ROS_INFO("Calculando odometría final...");
    odom_final();
    ros::Duration(0.5).sleep();

    // Escribir resultados
    ROS_INFO("Escribiendo resultados en archivo...");
    calcularYEscribir();

    // Finaliza el nodo ROS
    ros::shutdown();
    return 0;
    
}



