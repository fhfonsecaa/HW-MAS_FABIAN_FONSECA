/****************************************************
* Exercise1.h                                       *
* @author   Fabian Fonseca fhfonsecaa@gmail.com     *
* @date     Oct 2019                                *
****************************************************/

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(Exercise1,
{
	public:
		float _initMacro,
    (Vector2f)(0.f, 0.f) robotPose,
    (Vector2f)(0.f, 0.f) ballPose,
});



// class avc_filter{
// public:
//     /**
//      * Constructor del filtro de AVC el cual recibe aputnadores a publicadores y
//      * servicios que emplea en su funcionamiento, así como los parámetros de entrada
//      * empleados en la definición de fronteras del volumen de interés.
//      * @method avc_filter
//      * @param  theta               Ángulo de inclinacion del láser LMS empleado en
//      *                             la matriz de rotación
//      * @param  lim_y_p             Límite superior de la profundidad de escaneo
//      * @param  lim_y_n             Límite inferior de la profundidad de escaneo
//      * @param  lim_z_p             Límite superior de la altura de escaneo
//      * @param  lim_z_n             Límite inferior de la altura de escaneo
//      * @param  pubPointCloudP      Apuntador al publicador de la nube de puntos
//      * @param  pubPointCloudModelP Apuntador al publicador del modelo creado en nube de puntos
//      * @param  cliStartVidP        Apuntador al servicio que inicia la captura de video
//      * @param  cliStopVidP         Apuntador al servicio que termina la captura de video
//      */
//     avc_filter(std::string username,float theta,float lim_y_p,float lim_y_n,float lim_z_p
//       ,float lim_z_n,avc_classifier* classifierP,ros::Publisher* pubPointCloudP,ros::Publisher* pubPointCloudModelP
//       ,ros::ServiceClient* cliStartVidP,ros::ServiceClient* cliStopVidP,TCPClient* clientP);

//     /**
//      * Responde a los eventos desencadenados por la recepción de un mensaje en el
//      * tópico ls_msg producto del escaneo del láser LMS llamando la función de evaluación
//      * @method lsCallback
//      * @param  scan_msg   Mensaje codificado con vectores angulares e intensidades
//      * TODO    Revisar si se pueden obtener las intensidades desde el láser
//      */
//     void lsCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

//     /**
//      * Comparador de una sola dimensión para hacer el sorting de mayor a menor
//      * @method point1Dcomp
//      * @param  a           Primer elemento de la comparación
//      * @param  b           Segundo elemento de la comparación
//      */
//     static bool point1Dcomp(const float &a, const float &b)
//     {
//         return a > b;
//     }

//     /**
//      * Inicializa banderas, abre archivos, activa detonantes de captura de video
//      * y demás variables empleadas como asistencia para la segmentación de modelos
//      * @method startModeling
//      */
//     void startModeling();

//     /**
//      * Resetea las banderas, cierra archivos pendientes, da la orden de terminar
//      * la captura de video y da inicio a la clasificación del modelo culminado
//      * @method stopModeling
//      */
//     void stopModeling();

//     /**
//      * Evalua si el mensaje recibido contiene puntos de interes para el algoritmo,
//      * los filtra, les aplica la rotación y los unifica en un solo modelo de nube
//      * de puntos
//      * @method evaluate
//      * @param  pcl_msg  Mensaje recibido por el callback con la nube de puntos
//      * @param  plane    Conteo de planos recibidos por el láser LMS
//      * @return          Nube de puntos final con el modelo identificado
//      */
//     sensor_msgs::PointCloud evaluate(sensor_msgs::PointCloud pcl_msg);

//     sensor_msgs::PointCloud constructSlide(sensor_msgs::PointCloud pcl_msg, int plane);

//     bool commandCallBack(avc_id::command::Request &req, avc_id::command::Response &res);

//     void setStreamingEnabled(bool state);
    

//     /**
//      * Destructor por defecto
//      * @method ~avc_filter
//      */
//     virtual ~avc_filter();

// private:
//     /**Águlo de inclinación del láser LMS*/
//     float theta;
//     /**Límites del volumen de interés*/
//     float lim_x_p,lim_x_n,lim_y_p,lim_y_n,lim_z_p,lim_z_n;
//     /**Velocidad esperada del vehículo*/
//     float vv;
//     /**Delta de desplazamiento en direccion X*/
//     float dx;
// };
// #endif
