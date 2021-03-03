package database

import (
	"context"
	"github.com/fatih/color"
	"github.com/gwaxG/robot_ws/backend/internal/common"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
	"log"
)

const _DBAddr = "mongodb://localhost:27017"

type DataBase struct {
	client  		*mongo.Client
	database		*mongo.Database
	collection 		*mongo.Collection
	ExpSeriesName	string
	CollectionName	string
}

func (db *DataBase) Init () {
	var err error
	db.client, err = mongo.Connect(context.TODO(), options.Client().ApplyURI("mongodb://127.0.0.1:27017"))
	if err == nil {
		color.Green("Connected to MongoDB")
	} else {
		color.Red("Check if `sudo systemctl start mongod` was executed")
		common.FailOnError(err)
	}
}

func (db *DataBase) changeDatabase (databaseName string) {
	db.database = db.client.Database(databaseName)
}

func (db *DataBase) changeCollection (collectionName string) {
	db.collection = db.database.Collection(collectionName)
}

func (db *DataBase) check (analytics *structs.RolloutAnalytics) {
	dbName := analytics.Experiment
	collName := analytics.Exp
	if db.database == nil {

	}

}

func (db *DataBase) AddNewRolloutAnalytics(analytics structs.RolloutAnalytics) {
	db.check(&analytics)
	insertResult, err := db.collection.InsertOne(context.TODO(), analytics)
	log.Println("Inserted a single rollout: ", insertResult.InsertedID)
	common.FailOnError(err)
}

func (db *DataBase) Close() {
	db.client.Disconnect(context.TODO())
}