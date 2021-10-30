/*
 * Copyright 2012 - 2018 Anton Tananaev (anton@traccar.org)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.traccar.protocol;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.channel.Channel;
import org.traccar.BaseProtocolDecoder;
import org.traccar.DeviceSession;
import org.traccar.Protocol;
import org.traccar.helper.BitUtil;
import org.traccar.model.Network;
import org.traccar.model.Position;
import org.traccar.model.WifiAccessPoint;

import java.net.SocketAddress;
import java.util.ArrayList;

public class XexunBinaryProtocolDecoder extends BaseProtocolDecoder {

    static class AlarmData
    {
        DataTypes alarmDataBits;
        boolean DataSet = false;
        public ByteBuf reader;
        public DataTypes getAlarmDataBits ()
        {
                if (!DataSet)
                {
                    byte[] alarmDataResult = reader.readBytes(4).array();

                    alarmDataBits.dataType = new boolean[AlarmDataType.values().length];
                    int enumCount = AlarmDataType.values().length;
                    for (int i = 0; i < enumCount; i++)
                    {
                        int j = i;
                        int k = i / 8;
                        if (i > 7)
                        {
                            j = i % 8;
                        }
                        alarmDataBits.dataType[i] = BitUtil.check(alarmDataResult[k], j);
                    }
                    DataSet = true;
                }
                return alarmDataBits;
        }

        public AlarmData (ByteBuf reader)
        {
            this.reader = reader;
        }

        public String toString()
    {
        StringBuilder result = new StringBuilder();
        if (DataSet)
        {
            for (AlarmDataType type : AlarmDataType.values())
            {
                if (alarmDataBits.dataType[type.label])
                {
                    result.append(type).append(" ");
                }
            }
        }

        return result.toString();

    }
    }
    static class PositionData
    {

        public class positionData
        {
            public short serial; //u8
            public long timestampRaw; //u32
            public short signalStrength; //u8
            public int batteryInfo; //u16
            public boolean charging;
            public DataTypes dataTypes;
            public GPS gpsData;
            public WIFI wifiData;
            public TOF tofData;
            public LBS lbsData;
            public DGPS dGpsData;
            public SPEED speedData;
        }

        ByteBuf reader;
        boolean DataRead = false;

        private positionData _positionData;
        public positionData getPositionData()
        {
                if (!DataRead)
                {
                    _positionData.serial = reader.readUnsignedByte();
                    _positionData.timestampRaw = reader.readUnsignedInt();
                    _positionData.signalStrength = reader.readUnsignedByte();
                    byte charging = reader.readByte();
                    _positionData.charging = charging != 0x00;
                    _positionData.batteryInfo = reader.readUnsignedByte();
                    //_positionData.positionDataType = readerp();
                    byte posDataBits = reader.readByte();
                    _positionData.dataTypes.dataType = new boolean[DataType.values().length];

                    for (DataType type : DataType.values()) {
                        _positionData.dataTypes.dataType[type.label] = BitUtil.check(posDataBits, type.label);
                    }
                }
                return _positionData;
        }

        public void getLocationData()
        {
            byte posData = reader.readByte();
            DataTypes positionDataType = _positionData.dataTypes;
            positionDataType.dataType = new boolean[PositionDataType.values().length];

            for (PositionDataType type : PositionDataType.values()) {
                positionDataType.dataType[type.label] = BitUtil.check(posData, type.label);
            }

            if (positionDataType.dataType[PositionDataType.GPS.label])
            {
                _positionData.gpsData = new GPS(reader);
            }

            if (positionDataType.dataType[PositionDataType.WIFI.label])
            {
                _positionData.wifiData = new WIFI(reader);
            }

            if (positionDataType.dataType[PositionDataType.LBS.label])
            {
                _positionData.lbsData = new LBS(reader);
            }

            if (positionDataType.dataType[PositionDataType.TOF.label])
            {
                _positionData.tofData = new TOF(reader);
            }

            if (positionDataType.dataType[PositionDataType.SPEED.label])
            {
                _positionData.speedData = new SPEED(reader);
            }

            if (positionDataType.dataType[PositionDataType.HighPrecisionGPS.label])
            {
                _positionData.dGpsData = new DGPS(reader);
            }

        }
        public PositionData(ByteBuf reader)
        {
            this.reader = reader;
        }


        public static class GPS
        {
            public static class GpsData
            {
                public short numSats;
                public Float longi;
                public Float lat;
                public double resultLong;
                public double resultLat;
            }
            public GpsData data = new GpsData();
            public GPS (ByteBuf reader)
            {
                data.numSats = reader.readUnsignedByte();
                data.longi = reader.readFloat();
                data.lat = reader.readFloat();
                data.resultLong = convertLat_lng(data.longi.toString());
                data.resultLat = convertLat_lng(data.lat.toString());
            }
        }
        public static class LBS
        {
            public class wifiData
            {
                public short numLBSSignals; //u8
                public int[] mcc; //u16
                public int[] mcn; //u16
                public long[] lac; //u32
                public long[] cid; //u32
                public short[] rssi; //u8
            }
            public wifiData data = new wifiData();

            public LBS(ByteBuf reader)
            {
                data.numLBSSignals = reader.readUnsignedByte();
                data.mcc = new int[data.numLBSSignals];
                data.mcn = new int[data.numLBSSignals];
                data.lac = new long[data.numLBSSignals];
                data.cid = new long[data.numLBSSignals];
                data.rssi = new short[data.numLBSSignals];

                for (int i = 0; i < data.numLBSSignals; i++)
                {
                    data.mcc[i] = reader.readUnsignedShort();
                    data.mcn[i] = reader.readUnsignedShort();
                    data.lac[i] = reader.readUnsignedInt();
                    data.cid[i] = reader.readUnsignedInt();
                    data.rssi[i] = reader.readUnsignedByte();
                }
            }

        }

        public static class WIFI
        {
            public static class wifiData
            {
                public short numberOFWifiLocations; //u8
                public String[] macAddress; //u32
                public short[] signalStrength; //u8
            }
            public wifiData data = new wifiData();

            public WIFI (ByteBuf reader)
            {
                data.numberOFWifiLocations = reader.readUnsignedByte();
                data.macAddress = new String[data.numberOFWifiLocations];
                data.signalStrength = new short[data.numberOFWifiLocations];

                for (int i = 0; i < data.numberOFWifiLocations; i++)
                {
                    data.macAddress[i] = String.format("%02x:%02x:%02x:%02x:%02x:%02x",
                            reader.readUnsignedByte(), reader.readUnsignedByte(), reader.readUnsignedByte(),
                            reader.readUnsignedByte(), reader.readUnsignedByte(), reader.readUnsignedByte());
                    data.signalStrength[i] = reader.readByte();
                }
            }

        }
        public static class TOF
        {
            public static class tofData
            {
                public short TOFnumbers;
                public long serialNum;
                public long time;
                public int distance;
                public byte chargingBit;
                public boolean isCharging;
                public short chargeLevel2;
            }

            public tofData data = new tofData();

            public TOF(ByteBuf reader)
            {
                data.TOFnumbers = reader.readUnsignedByte();//U8  1    TOFnumbers
                data.serialNum = reader.readUnsignedInt();//U32 4    Binding serial numb
                data.time = reader.readUnsignedInt();//U32 4    Update time
                data.distance = reader.readShort();//U16 2 Ranging distance(Base station battery
                data.chargingBit = reader.readByte();//U16 2 and charging status
                data.isCharging = BitUtil.check(data.chargingBit, 7);
                data.chargeLevel2 = reader.readUnsignedByte();
            }
        }
        public static class SPEED
        {
            public static class SpeedData
            {
                public int speed;
                public int direction;
            }

            public SpeedData data = new SpeedData();

            public SPEED(ByteBuf reader)
            {
                data.speed = reader.readUnsignedShort();
                data.direction = reader.readUnsignedShort();
            }
        }
        public static class DGPS
        {
            public static class DGpsData
            {
                public short numSats;
                public double resultLong;
                public double resultLat;
            }
            public DGpsData data = new DGpsData();

            public DGPS(ByteBuf reader)
            {
                data.numSats = reader.readUnsignedByte();
                data.resultLong = convertLat_lng(((Double)reader.readDouble()).toString());
                data.resultLat = convertLat_lng(((Double)reader.readDouble()).toString());
            }
        }


    }

    public static Double convertLat_lng(String lat_lng)
    {
        String degree;
        String min;
        if (lat_lng.equals("0.0"))
        {
            return 0d;
        }

        if (lat_lng.contains("."))
        {
            int index = lat_lng.indexOf(".");
            int degree_min_index = index > 2 ? index - 2 : index;
            degree = lat_lng.substring(0, degree_min_index);
            min = lat_lng.substring(degree_min_index);
        }
        else
        {
            int index = lat_lng.length() > 2 ? lat_lng.length() - 2 : 0;
            degree = lat_lng.substring(0, index);
            min = lat_lng.substring(index);
        }

        return Double.parseDouble(degree) + Double.parseDouble(min) / 60;
    }
    static class ExtendedData
    {
        private final boolean dataSet;

        private final healthData _healthData = new healthData();
        public healthData getHealthData()
        {
                if (dataSet)
                { return _healthData; }
                return null;
        }

        public static class healthData
        {
            public short hr;
            public short lowP;
            public short highP;
            public int steps;
            public short o2;
        }

        public static class tempData
        {
            public float aTemp;
            public float rawTemp;
            public float correctionValue;
        }

        private final tempData _tempData = new tempData();
        public tempData getTempData()
        {
                if (dataSet)
                { return _tempData; }
                return null;
        }

        public ExtendedData (ByteBuf reader)
        {
            reader.skipBytes(3); //throw away junk data
            byte exData = reader.readByte();
            boolean tempOnly = BitUtil.check(exData, 0);
            boolean humanBodyData = BitUtil.check(exData, 1);

            if (tempOnly)
            {
                _tempData.aTemp = reader.readFloat();
                _tempData.rawTemp = reader.readFloat();
                _tempData.correctionValue = reader.readFloat();
            }
            else if (humanBodyData)
            {
                _healthData.hr = reader.readUnsignedByte();
                _healthData.lowP = reader.readUnsignedByte();
                _healthData.highP = reader.readUnsignedByte();
                _healthData.steps = reader.readUnsignedShort();
                _healthData.o2 = reader.readUnsignedByte();
            }

            dataSet = true;
        }
    }

    static class TOFData
    {
        private boolean dataSet;

        private final tofData _tofData = new tofData();

        public tofData getTOFData()
        {
             if (dataSet)
             { return _tofData; }
             return null;


        }

        public static class tofData
        {
            public short timezone;
            public int positioningInt;
            public short packetSendingInt;
            public short bindNum;
            public short startTime;
            public short endTime;
            public short rangingDistance;
            public long bindingID;
        }


        public TOFData(ByteBuf reader)
        {
            _tofData.timezone = reader.readUnsignedByte();
            _tofData.positioningInt = reader.readUnsignedShort();
            _tofData.packetSendingInt = reader.readUnsignedByte();
            _tofData.bindNum = reader.readUnsignedByte();
            _tofData.startTime = reader.readUnsignedByte();
            _tofData.endTime = reader.readUnsignedByte();
            _tofData.rangingDistance = reader.readUnsignedByte();
            _tofData.bindingID = reader.readUnsignedInt();

            dataSet = true;
        }
    }
    enum DataType {
        AlarmData(0),
        PositionData(1),
        NA(2),
        FingerprintData(3),
        VersionData(4),
        DevicesTofParameters(5),
        NFCData(6),
        ExtendedData(7);

        public final int label;

        DataType(int label) {
            this.label = label;
        }
    }

    enum PositionDataType {
        GPS(0),
        WIFI(1),
        LBS(2),
        TOF(3),
        NA(4),
        SPEED(5),
        HighPrecisionGPS(6);
        public final int label;

        PositionDataType(int label) {
            this.label = label;
        }
    }

    enum AlarmDataType {
        SOS(0), //SOS
        Dismantle(1),// Dismantle alarm bit
        NA(2),// N/A
        NA2(3),// N/A
        Charging(4),//charging bit
        NA3(5),//N/A
        TurnOn(6),//Turn on
        HeartRate(7),//  State of the heart ra bit
        PIN(8),//PIN
        CheckSwitch(9),//Disconnect
        Connection(10),// Connection
        GravityFall(11),// Gravity alarm switc bit
        Movement(12),// Movement
        Stasic(13),// Static bit
        NA4(14), //    N/A
        FallDown(15),//   Fall down alarm bit
        OpticalSensor(16),// Optical sensors sw bit
        NotTouched(17),// Not touched
        Touched(18),//    Touch
        AnkleBraceletOffline(19),// Ankle bracelet tags offline
        AnkleBraceletOverDistance(20),// Ankle bracelet tags over
        OutOfGeofence(21),//    Coordinates out o bounds
        CarOff(22);//    Car power down
        public final int label;


        AlarmDataType(int label) {
            this.label = label;
        }
    }

        public static class DataTypes {
            public boolean[] dataType;
        }

    public XexunBinaryProtocolDecoder(Protocol protocol) {
        super(protocol);
    }

    /**
     * Decodes an unsigned packed BCD to its integer number wrapped in a {@code String}.
     *
     * @param bcd the BCD to decode.
     * @return the decoded integer number wrapped inside a {@code String}.
     */
    public static String bcdToString(byte[] bcd) {
        StringBuilder sb = new StringBuilder();

        for (byte b : bcd) {
            sb.append(bcdToString(b));
        }

        return sb.toString();
    }

    /**
     * Decodes an unsigned packed BCD byte to its integer number wrapped in a {@code String}.
     *
     * @param bcd the BCD byte to decode.
     * @return the decoded integer number wrapped inside a {@code String}.
     */
    public static String bcdToString(byte bcd) {
        StringBuilder sb = new StringBuilder();

        byte high = (byte) (bcd & 0xf0);
        high >>>= (byte) 4;
        high = (byte) (high & 0x0f);
        byte low = (byte) (bcd & 0x0f);

        sb.append(high);
        sb.append(low);

        return sb.toString();
    }
    public static int checksum(byte[] data, int len)
    {
        int sum = 0;
        for (int j = 0; len > 1; len--)
        {
            sum += data[j++] & 0xff;
            if ((sum & 0x80000000) > 0)
            {
                sum = (sum & 0xffff) + (sum >> 16);
            }
        }
        if (len == 1)
        {
            sum += data[data.length - 1] & 0xff;
        }
        while ((sum >> 16) > 0)
        {
            sum = (sum & 0xffff) + sum >> 16;
        }
        sum = (sum == 0xffff) ? sum & 0xffff : (~sum) & 0xffff;
        return sum;
    }
    public static int byteArrayToLeInt(byte[] encodedValue) {
        int value = (encodedValue[1] & 0xFF) << (Byte.SIZE);
        value |= (encodedValue[0] & 0xFF);
        return value;
    }

    @Override
    protected Object decode(
            Channel channel, SocketAddress remoteAddress, Object msg) throws Exception {

        ByteBuf buf = (ByteBuf) msg;

        buf.skipBytes(2); // header


        Position position = new Position(getProtocolName());

        buf.readUnsignedShort(); // messageId
        position.set(Position.KEY_INDEX, buf.readUnsignedShort()); // serial number of message;
/*        position.set(Position.bcdToString(buf.readBytes(8).array());
        // Zero for location messages
        int power = buf.readUnsignedByte();
        int gsm = buf.readUnsignedByte();*/

        String imei = String.format("%015d", buf.readLongLE());
        DeviceSession deviceSession = getDeviceSession(channel, remoteAddress, imei);
        if (deviceSession == null) {
            return null;
        }
        position.setDeviceId(deviceSession.getDeviceId());

        //position.set(Position.KEY_INDEX, buf.readUnsignedShort());
        byte[] mask = { (byte)0xFF, (byte)0xC0 };
        int msgSize = buf.readUnsignedShort();
        msgSize &= byteArrayToLeInt(mask);
        int index = buf.readerIndex();
        byte[] msgForECC = new byte[msgSize];
        int checksumVal = buf.readUnsignedShort();
        buf.readBytes(msgForECC);
        buf.readerIndex(index);
        int checksumRetVal = checksum(msgForECC, msgSize);

        short dataNum = buf.readUnsignedByte();
        int[] sizes = new int[dataNum];

        for (int i = 0; i < dataNum; i++)
        {
            sizes[i] = buf.readUnsignedShort();
        }

        ArrayList<byte[]> packets = new ArrayList<byte[]>();
        for (int i = 0; i < dataNum; i++)
        {
            byte[] aPacket = new byte[sizes[i]];
            buf.readBytes(aPacket);
            packets.add(aPacket);
        }
        ArrayList<ByteBuf> bufPackets = new ArrayList<>();
        for (byte[] packet : packets) {
            bufPackets.add(Unpooled.wrappedBuffer(packet));
        }

        PositionData aPosition = new PositionData(bufPackets.get(0));
        var dataType = aPosition.getPositionData();
        ExtendedData extendedData;
        TOFData tofData;

        if (dataType.dataTypes.dataType[DataType.AlarmData.label])
        {
            AlarmData alarmData = new AlarmData(bufPackets.get(0));
            var data = alarmData.alarmDataBits;
            var result = alarmData.getAlarmDataBits().dataType;
            for (AlarmDataType alarm : AlarmDataType.values()) {
                if (alarmData.getAlarmDataBits().dataType[alarm.label]) {
                    position.set(Position.KEY_ALARM, alarm.toString());
                }
            }
        }

        if (dataType.dataTypes.dataType[DataType.PositionData.label]) {
            aPosition.getLocationData();
            aPosition.getPositionData();

            if (aPosition.getPositionData().dataTypes.dataType[PositionDataType.GPS.label]) {
                position.setLatitude(aPosition.getPositionData().gpsData.data.resultLat);
                position.setLongitude(aPosition.getPositionData().gpsData.data.resultLong);
                position.set(Position.KEY_SATELLITES, aPosition.getPositionData().gpsData.data.numSats);
            }
            if (aPosition.getPositionData().dataTypes.dataType[PositionDataType.WIFI.label]) {
                Network network = new Network();
                for (int i =0; i < aPosition.getPositionData().wifiData.data.numberOFWifiLocations; i++) {
                    WifiAccessPoint wifiAccessPoint = new WifiAccessPoint();
                    wifiAccessPoint.setMacAddress(aPosition.getPositionData().wifiData.data.macAddress[i]);
                    wifiAccessPoint.setSignalStrength((int) aPosition.getPositionData().wifiData.data.signalStrength[i]);
                    network.addWifiAccessPoint(wifiAccessPoint);
                }
            }
        }
        if (dataType.dataTypes.dataType[DataType.VersionData.label])
        {
            //string versionData = new String(posReader.ReadChars(20));
            //string imsi = posReader.FromBCDToExtUInt64();
            //string iccid = posReader.FromBCDToExtUInt64(10);
        }

        if (dataType.dataTypes.dataType[DataType.DevicesTofParameters.label])
        {
            tofData = new TOFData(bufPackets.get(0));
        }

        if (dataType.dataTypes.dataType[DataType.ExtendedData.label])
        {
            extendedData = new ExtendedData(bufPackets.get(0));
            Object healthData = extendedData.getHealthData();
            if (healthData == null)
            {
                healthData = extendedData.getTempData();
            } else {
                position.set(Position.KEY_HEART_RATE, ((ExtendedData.healthData)healthData).hr);
            }
        }


        return position;
    }

}
