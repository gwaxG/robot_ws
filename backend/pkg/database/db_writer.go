package database

import (
	"context"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"github.com/gwaxG/robot_ws/monitor/pkg/structs"
	"log"
)

// Append new rollout analytics to corresponding database and collection
func (db *DataBase) AddNewRolloutAnalytics(analytics structs.RolloutAnalytics) {
	db.check(analytics.ExpSeries, analytics.Experiment)
	log.Printf("Inserting new to db %s collection %s\n",  db.database.Name(), db.collection.Name())
	db.mutex.Lock()
	insertResult, err := db.collection.InsertOne(context.TODO(), analytics)
	db.mutex.Unlock()
	common.FailOnError(err)
	log.Println("Successfully inserted a single rollout: ", insertResult.InsertedID)
}
