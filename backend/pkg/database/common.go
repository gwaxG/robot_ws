package database

import (
	"context"
	"github.com/fatih/color"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
	"sync"
)

const _DBAddr = "mongodb://localhost:27017"

type DataBase struct {
	client  		*mongo.Client
	database		*mongo.Database
	collection 		*mongo.Collection
	mutex 			*sync.Mutex
}

// Connect to the mongodb server
func (db *DataBase) Init () {
	var err error
	db.client, err = mongo.Connect(context.TODO(), options.Client().ApplyURI(_DBAddr))
	if err == nil {
		color.Green("Connected to MongoDB")
	} else {
		color.Red("Check if `sudo systemctl start mongod` was executed")
		common.FailOnError(err)
	}
	db.mutex = &sync.Mutex{}
}

// Change database
func (db *DataBase) changeDatabase (databaseName string) {
	db.database = db.client.Database(databaseName)
}

// Change collection
func (db *DataBase) changeCollection (collectionName string) {
	db.collection = db.database.Collection(collectionName)
}

// Check if the used database and collection correspond to the received analytics.
func (db *DataBase) check (dbName, collName string) {
	if db.database == nil || db.database.Name() != dbName {
		db.changeDatabase(dbName)
	}
	if db.collection == nil || db.collection.Name() != collName {
		db.changeCollection(collName)
	}
}


// Disconnect client from the DB server
func (db *DataBase) Close() {
	db.client.Disconnect(context.TODO())
}

