package database

import (
	"context"
	"log"

	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
	"go.mongodb.org/mongo-driver/bson"
)

// Append new rollout analytics to corresponding database and collection
func (db *DataBase) AddNewRolloutAnalytics(analytics structs.RolloutAnalytics) {
	db.check(analytics.ExpSeries, analytics.Experiment)
	log.Printf("Inserting new to db %s collection %s\n", db.database.Name(), db.collection.Name())
	db.mutex.Lock()
	insertResult, err := db.collection.InsertOne(context.TODO(), analytics)
	db.mutex.Unlock()
	common.FailOnError(err)
	log.Println("Successfully inserted a single rollout: ", insertResult.InsertedID)
}

// save config to collection HistoryConfigs in the form of document {name: string, config: json}
func (db *DataBase) SaveConfig(Config map[string]interface{}) {
	defer func() {
		if r := recover(); r != nil {
			log.Println(
				"Recoverd in db_writer.SaveConfig on",
				Config["experiment_series"].(string),
				Config["experiment"].(string))
		}
	}()
	ExpSeries := Config["experiment_series"].(string)
	Experiment := Config["experiment"].(string)
	db.check(ExpSeries, "HistoryConfigs")

	// check if the experiment exists in the database
	var exist = false
	cursor, err := db.collection.Find(context.TODO(), bson.M{})
	if err != nil {
		log.Fatal(err)
	}
	defer cursor.Close(context.TODO())
	for cursor.Next(context.TODO()) {
		var doc bson.M
		if err = cursor.Decode(&doc); err != nil {
			log.Fatal(err)
		}
		if doc["name"] == Experiment {
			exist = true
			break
		}
	}
	// insert a new experiment or update one
	if exist {
		// Update
		_, err := db.collection.UpdateOne(
			context.TODO(),
			bson.M{"name": Experiment},
			bson.D{
				{"$set", bson.D{{"config", Config}}},
			},
		)
		if err != nil {
			log.Panic(err)
		}
	} else {
		req := map[string]interface{}{
			"config": Config,
			"name":   Experiment,
		}
		// Create
		_, err := db.collection.InsertOne(context.TODO(), req)
		if err != nil {
			log.Panic(err)
		}
	}
	_ = Experiment
}

// delete a config from collection HistoryConfigs
func (db *DataBase) DeleteConfig(ExperimentSeries, Experiment string) {
	defer func() {
		if r := recover(); r != nil {
			log.Println(
				"Recoverd in db_writer.DeleteConfig on",
				ExperimentSeries,
				Experiment)
		}
	}()
	db.check(ExperimentSeries, "HistoryConfigs")

	// check if the experiment exists in the database
	var exist = false
	cursor, err := db.collection.Find(context.TODO(), bson.M{})
	if err != nil {
		log.Fatal(err)
	}
	defer cursor.Close(context.TODO())
	for cursor.Next(context.TODO()) {
		var doc bson.M
		if err = cursor.Decode(&doc); err != nil {
			log.Fatal(err)
		}
		if doc["name"] == Experiment {
			exist = true
			break
		}
	}
	// insert a new experiment or update one
	if exist {
		// Update
		_, err := db.collection.DeleteOne(context.TODO(), bson.M{"name": Experiment})
		if err != nil {
			log.Panic(err)
		}
	}
}
