#include "application.h"

#include <ros.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>

#include "../src/Transaction.h"
#include "./helpers/CryptoHelper.h"
#include "./helpers/ByteConverter.h"

#include <vector>

static const uint8_t PRIVATE_KEY[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                      0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                      0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                      0x01, 0x01};

#define CONTRACT_ADDRESS "0xced3564e187544294bd88059ae0456a912b460ad"
#define STRING_TYPE_PREFIX "0000000000000000000000000000000000000000000000000000000000000020"
#define COORDS_LENGHT "000000000000000000000000000000000000000000000000000000000000000f"
#define METHOD_ADDRESS "0x45eb183f"
#define CHAIN_ID 3 // 0x1-mainnet, 0x3-Ropsten, 0x4-Rinkeby
#define GAS_PRICE 50000000000
#define GAS_LIMIT 100000

ros::NodeHandle rosNode;

string checksumAddressHex;
string currentCoords = "";

std_msgs::String blockCountCommand;
std_msgs::String smartContractCommand;

ros::Publisher transactionCountRequestPublisher("transactionCountRequest", &blockCountCommand);
ros::Publisher smartContractRequestPublisher("smartContractRequest", &smartContractCommand);

void transactionCountCallback(const std_msgs::String& transactionCountMessage)
{
  Serial.printlnf("Transaction count: %s", transactionCountMessage.data);

  if (currentCoords != "")
  {
    vector<uint8_t> rawCoords = ByteConverter::stringToBytes(&currentCoords);

    // Removing the leading 0x
    String hexCoords = ByteConverter::bytesToString(rawCoords).substring(2);

    // Now we can start building up the transaction itself
    Transaction transaction;
    transaction.setPrivateKey((uint8_t*)PRIVATE_KEY);

    // Make sure that nonce number is not smaller than the total number of transactions for the address
    // Total transaction is available through JSON-RPC method eth_getTransactionCount
    uint32_t nonce = atoi(transactionCountMessage.data);
    uint32_t chainId = CHAIN_ID;
    uint32_t gasPrice = GAS_PRICE;
    uint32_t gasLimit = GAS_LIMIT;

    string to = CONTRACT_ADDRESS;
    string value = "";
    string data = string(METHOD_ADDRESS).append(STRING_TYPE_PREFIX).append(COORDS_LENGHT).append(hexCoords);

    Serial.printlnf("Data: %s", data.c_str());

    // The last step is passing all the necessary values for transaction signing process
    String rawTransaction = transaction.getRaw(nonce, gasPrice, gasLimit, &to, &value, &data, chainId);

    // Raw transaction is ready to be sent to JSON-RPC method call eth_sendRawTransaction
    Serial.printlnf("Signed raw transaction: %s", rawTransaction.c_str());

    smartContractCommand.data = rawTransaction.c_str();
    smartContractRequestPublisher.publish(&smartContractCommand);
  }
}

void gpsLocationCallback(const nav_msgs::Odometry& gpsLocationMessage)
{
  string xCoord = string(String(gpsLocationMessage.pose.pose.position.x, 4));
  string yCoord = string(String(gpsLocationMessage.pose.pose.position.y, 4));

  currentCoords = xCoord + ":" + yCoord;

  // Making sure that coords are ready for RLP encoding.
  while(currentCoords.length() <= 64)
  {
    currentCoords.append("0");
  }
}

ros::Subscriber<std_msgs::String> transactionCountSubscriber("transactionCountResult", &transactionCountCallback);
ros::Subscriber<nav_msgs::Odometry> gpsLocationSubscriber("gpsLocation", &gpsLocationCallback);

void setup()
{
  Serial.begin(9600);

  // First, we need to create public key from the private one
  vector<uint8_t> publicKey = CryptoHelper::generateAddress(PRIVATE_KEY);

  // Then, we will create public address using public key
  vector<char> checksumAddress = CryptoHelper::generateChecksumAddress(publicKey);

  // Let's convert the address into HEX format
  checksumAddressHex = string(checksumAddress.begin(), checksumAddress.end());

  Serial.printlnf("Wallet address: %s", checksumAddressHex.c_str());

  IPAddress rosCoreAddress(192, 168, 5, 240);
  int rosCorePort = 11411;

  rosNode.getHardware()->setConnection(rosCoreAddress, rosCorePort);
  rosNode.initNode();

  rosNode.advertise(transactionCountRequestPublisher);
  rosNode.advertise(smartContractRequestPublisher);

  rosNode.subscribe(transactionCountSubscriber);
  rosNode.subscribe(gpsLocationSubscriber);
}

void loop()
{
  Serial.printlnf("Sending transaction count request.");

  blockCountCommand.data = checksumAddressHex.c_str();
  transactionCountRequestPublisher.publish(&blockCountCommand);

  rosNode.spinOnce();

  delay(5000);
}
