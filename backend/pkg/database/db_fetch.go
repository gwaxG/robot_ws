package database

import (
	"context"
	"encoding/json"
	"errors"
	"github.com/gwaxG/robot_ws/backend/pkg/common"
	"go.mongodb.org/mongo-driver/bson"
	"io"
	"io/ioutil"
	"log"
	"strings"
)

type ResponseDbs struct {
	Data	[]string `json:"data"`
	Msg		string   `json:"msg"`
}

// Append new rollout analytics to corresponding database and collection
func (db *DataBase) FetchDbs() (*ResponseDbs, error){
	res, err := db.client.ListDatabaseNames(context.TODO(), bson.D{})
	if err != nil {
		return nil, errors.New("can not fetch database names")
	}
	out := []string{}
	for i:=0; i<len(res); i++ {
		if strings.Contains(res[i], "exp") {
			out = append(out, res[i])
		}
	}
	msg := &ResponseDbs{
		Data: out,
		Msg:  "",
	}
	return msg, nil
}

type ResponseColls struct {
	Colls	[]string `json:"colls"`
	Fields 	[][]string `json:"fields"`
	Msg		string 	 `json:"msg"`
}

// Retrieve collections from a database with associated fields
func (db *DataBase) FetchCollections(database string) (*ResponseColls, error){
	msg := &ResponseColls{
		Colls: []string{},
		Fields: [][]string{},
		Msg:  "",
	}
	// check if database exists
	res, err := db.client.ListDatabaseNames(context.TODO(), bson.D{})
	if err != nil {
		return nil, errors.New("can not fetch database names")
	}
	exists := false
	for i:=0; i<len(res); i++ {
		if strings.Contains(res[i], database) {
			exists = true
		}
	}
	if !exists {
		msg.Msg = "Database does not exist"
		return msg, nil
	}
	db.changeDatabase(database)
	collNames, err := db.database.ListCollectionNames(context.TODO(), bson.D{})
	if err != nil {
		return nil, errors.New("can not fetch collection names")
	}
	for _, collName := range collNames {
		if collName == "HistoryConfigs" {
			continue
		}
		msg.Colls = append(msg.Colls, collName)
		var fields []string
		db.changeCollection(collName)
		doc := map[string]interface{}{}
		common.FailOnError(db.collection.FindOne(context.TODO(), bson.M{}).Decode(&doc))
		for fieldName, _ := range doc {
			if fieldName != "_id" {
				fields = append(fields, fieldName)
			}
		}
		msg.Fields = append(msg.Fields, fields)
	}
	return msg, nil
}

type RequestVisualize struct {
	Data	[]string 	`json:"data"`
}

type ResponseVisualize struct {
	Data	map[string][]float64 	`json:"data"`
	Msg		string 	 				`json:"msg"`
}

/*
localhost:10000/visualize?database=test_exps&collection=test_rollout&fields=reward_progress_cogheight
*/
func (db *DataBase) FetchVisualize(dbName, collName string, reqRaw io.ReadCloser) (*ResponseVisualize, error){
	defer func (){
		if r := recover(); r!=nil{log.Println("Recovered in db_fetch.FetchVisualize")}
	}()
	msg := ResponseVisualize{
		Data: map[string][]float64{},
		Msg: "",
	}
	fields := RequestVisualize{}
	body, err := ioutil.ReadAll(reqRaw)
	if err != nil {
		return &msg, err
	}
	err = json.Unmarshal(body, &fields)
	if err != nil {
		return &msg, err
	}
	db.changeDatabase(dbName)
	db.changeCollection(collName)
	cursor, err := db.collection.Find(context.TODO(), bson.M{})
	common.FailOnError(err)
	var docs []bson.M
	err = cursor.All(context.TODO(), &docs)
	common.FailOnError(err)
	for _, doc := range docs {
		for k, v := range doc {
			if contains(fields.Data, k) {
				msg.Data[k] = append(msg.Data[k], v.(float64))
			}

		}
	}
	return &msg, nil
}

func contains(fields []string, field string) bool {
	for _, v := range fields {
		if v == field {
			return true
		}
	}
	return false
}

type ResponseHistoryConfig struct {
	Configs	[]map[string]interface{} `json:"configs"`
	Msg string `json:"msg"`
}

// fetch docs from a collection HistoryConfigs
func (db *DataBase) FetchHistoryConfig(dbName string) (*ResponseHistoryConfig, error){
	defer func (){
		if r := recover(); r!=nil{
		}
	}()
	msg := ResponseHistoryConfig{
		Configs: []map[string]interface{}{},
		Msg: "",
	}
	db.check(dbName, "HistoryConfigs")
	cursor, err := db.collection.Find(context.TODO(), bson.M{})
	if err != nil {
		log.Fatal(err)
	}
	defer cursor.Close(context.TODO())
	for cursor.Next(context.TODO()) {
		var doc bson.D
		var convertedDoc map[string]interface{}
		if err = cursor.Decode(&doc); err != nil {
			log.Fatal(err)
		}
		tempBytes, err := bson.MarshalExtJSON(doc, true, true)
		if err != nil {
			log.Fatal(err)
		}
		err = json.Unmarshal(tempBytes, &convertedDoc)
		if err != nil {
			log.Fatal(err)
		}
		msg.Configs = append(msg.Configs, convertedDoc)
	}
	return &msg, nil
}