
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.HAL;
import frc.robot.drivers.SwerveModule;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;


/**
 * Testing class for a swerve module.
 */
public class SwerveModuleTest {
    SwerveModule m_swerveModule;
    CANSparkMax m_turnMotor;

    @Before // this method will run before each test
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        m_swerveModule = new SwerveModule("Test Module", 0, 1, 0, 1.23, false);
    }

    @After // this method will run after each test
    public void shutdown() throws Exception {
        m_swerveModule.close();
        System.out.println("Close");
    }

    @Test // marks this method as a test
    public void hello_World() throws Exception {
        System.out.print("Hello ");
        assertEquals("World", m_swerveModule.helloWorld("World"));
    }

    @Test // Checks that Capsensitive fails if not Equal
    public void not_Hello_World() {
        System.out.print("Hello ");
        assertNotEquals("World", m_swerveModule.helloWorld("world"));
    }
}