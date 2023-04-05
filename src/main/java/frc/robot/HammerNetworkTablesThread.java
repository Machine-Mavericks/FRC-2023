package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;



    // hammer network tables
public class HammerNetworkTablesThread extends Thread {
    NetworkTable m_table;
    DoublePublisher dblPub;
    IntegerPublisher intPub0;
    IntegerPublisher intPub1;
    IntegerPublisher intPub2;
    IntegerPublisher intPub3;
    IntegerPublisher intPub4;
    IntegerPublisher intPub5;
    IntegerPublisher intPub6;
    IntegerPublisher intPub7;
    IntegerPublisher intPub8;

    public void run() {
        System.out.println("**********testing from thread*****************");

     

                hammertime();
    }
    private void hammertime(){
        m_table = NetworkTableInstance.getDefault().getTable("NTNightmare");    

        

        // get a topic from a NetworkTable
        // the topic name in this case is the name within the table;
        // this line and the one above reference the same topic
        DoubleTopic dblTopic = m_table.getDoubleTopic("X");
        IntegerTopic intTopic0 = m_table.getIntegerTopic("intY0");
        IntegerTopic intTopic1 = m_table.getIntegerTopic("intY1");
        IntegerTopic intTopic2 = m_table.getIntegerTopic("intY2");
        IntegerTopic intTopic3 = m_table.getIntegerTopic("intY3");
        IntegerTopic intTopic4 = m_table.getIntegerTopic("intY4");
        IntegerTopic intTopic5 = m_table.getIntegerTopic("intY5");
        IntegerTopic intTopic6 = m_table.getIntegerTopic("intY6");
        IntegerTopic intTopic7 = m_table.getIntegerTopic("intY7");
        IntegerTopic intTopic8 = m_table.getIntegerTopic("intY8");

        dblPub = dblTopic.publish();
        intPub0 = intTopic0.publish();
        intPub1 = intTopic1.publish();
        intPub2 = intTopic2.publish();
        intPub3 = intTopic3.publish();
        intPub4 = intTopic4.publish();
        intPub5 = intTopic5.publish();
        intPub6 = intTopic6.publish();
        intPub7 = intTopic7.publish();
        intPub8 = intTopic8.publish();
        

        long i = 0;
        while(true){
             // publish a value with current timestamp
                 dblPub.set(1.0);
                 intPub0.set(i);
                 intPub1.set(i);
                 intPub2.set(i);
                 intPub3.set(i);
                 intPub4.set(i);
                 intPub5.set(i);
                 intPub6.set(i);
                 intPub7.set(i);
                 intPub8.set(i);
                 i++;

                //  try {
                //     sleep(20);   
                //  } catch (Exception e) {
                //     System.out.println("can't sleep");
                //  }
                 

        }
    }

}