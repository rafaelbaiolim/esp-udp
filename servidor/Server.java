
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.util.Random;

public class Server {


    public static String ipSocket = "192.168.25.124";
    public static int serverPort = 9876;
    public static int arduinoPort = 9875;

    public static final String ON_MOTOR_1 = "ON_MOTOR_1";
    public static final String ON_MOTOR_2 = "ON_MOTOR_2";
    public static final String ON_MOTOR_3 = "ON_MOTOR_3";
    public static final String OFF_MOTOR_1 = "OFF_MOTOR_1";

    /**
     * @breath Envia um comando para o arduíno via UDP este método pode ser
     * implementado junto com os comandos de interface do projeto, utilize
     * depois que o servidor java estiver iniciado
     * @param comando Comando a ser executado
     * @param socket Socket de envio
     * @param IP_Arduino IP do socket
     * @throws IOException
     */
    public static void enviarComandoArduino(DatagramSocket socket,
            InetAddress IP_Arduino, String comando)
            throws IOException {
        System.out.println("\n>ENVIANDO COMANDO: " + comando);
        byte[] comandAsByte = comando.getBytes();

        DatagramPacket sendPacket = new DatagramPacket(comandAsByte,
                comandAsByte.length, IP_Arduino, arduinoPort);
        socket.send(sendPacket);
    }

    public static void main(String args[]) throws Exception {

        final String[] comandos = {ON_MOTOR_1, ON_MOTOR_2, ON_MOTOR_3, OFF_MOTOR_1};
        Random random = new Random();

        //Servidor
        DatagramSocket serverSocket = new DatagramSocket(null);
        InetSocketAddress bindAdress = new InetSocketAddress(ipSocket, serverPort);
        serverSocket.bind(bindAdress);
	System.out.println("Servidor Iniciado.");
        byte[] receiveData = new byte[1024];
        byte[] sendData = new byte[1024];

        while (true) {
            String sentence = "";
            //recebe o datagrama do cliente
            DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
            serverSocket.receive(receivePacket);
            sentence = new String(receivePacket.getData(), receivePacket.getOffset(), receivePacket.getLength());
            System.out.println(">>RECEIVED: " + sentence);

            //trecho exemplo de requisições para o Arduíno, pode ser removido
            //envia um comando aletório depois de receber os dados do ultrassonico:
            try{
                Float.parseFloat(sentence);
                enviarComandoArduino(serverSocket, receivePacket.getAddress(),
                        comandos[random.nextInt(comandos.length)]);
            }catch(Exception ex){
		      //só envia um comando quando recebe a distância do ultrassonico
            } 
        }
    }

}
