/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/
#ifndef APP_H
#define APP_H

class App {
    public:

        App();
        /**
        * Callback executed when the joystick connects to the board.
        */
        void setup(void);

        static  void onConnect();

        /**
        * Return by interface in the array that was indicated as parameter what was sent from the app. Format: [pitch,yaw].
        */
        void parse_input(float pitch_yaw[2] );

        /**
        * Adjusts the read pitch and returns it .
        *
        * @return Pitch target
        */
        float get_pitch_command(float read_n);

        /**
        * Get the commanded yaw value from the PS3 joystick.
        *
        * @param  {float} prev_target : Previous commanded yaw.
        * @return New commanded yaw.
        */
        float get_yaw_command(float prev_target, float read);

        void stopped_command(bool has_fallen);
};
#endif
